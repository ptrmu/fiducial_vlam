
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "fiducial_math.hpp"
#include "map.hpp"
#include "observation.hpp"
#include "vloc_context.hpp"

#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// VlocNode class
// ==============================================================================

  class VlocNode : public rclcpp::Node
  {
    VlocContext cxt_;
    std::unique_ptr<Map> map_{};
    std::unique_ptr<CameraInfo> camera_info_{};
    std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_{};
    Localizer localizer_{map_};
    std_msgs::msg::Header::_stamp_type last_image_stamp_{};

    rclcpp::Publisher<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_pub_{};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_{};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr base_pose_pub_{};
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_{};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr camera_odometry_pub_{};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odometry_pub_{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_{};

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;


  public:
    VlocNode(const rclcpp::NodeOptions &options)
      : Node("vloc_node", options), cxt_{*this}
    {
      // Get parameters from the command line
      cxt_.load_parameters();

      // ROS publishers. Initialize after parameters have been loaded.
      observations_pub_ = create_publisher<fiducial_vlam_msgs::msg::Observations>(
        cxt_.fiducial_observations_pub_topic_, 16);

      if (cxt_.publish_camera_pose_) {
        camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          cxt_.camera_pose_pub_topic_, 16);
      }
      if (cxt_.publish_base_pose_) {
        base_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          cxt_.base_pose_pub_topic_, 16);
      }
      if (cxt_.publish_tfs_ || cxt_.publish_tfs_per_marker_) {
        tf_message_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
          "/tf", 16);
      }
      if (cxt_.publish_camera_odom_) {
        camera_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          cxt_.camera_odometry_pub_topic_, 16);
      }
      if (cxt_.publish_base_odom_) {
        base_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          cxt_.base_odometry_pub_topic_, 16);
      }
      if (cxt_.publish_image_marked_) {
        image_marked_pub_ = create_publisher<sensor_msgs::msg::Image>(
          cxt_.image_marked_pub_topic_, 16);
      }

      // ROS subscriptions
      auto camera_info_qos = cxt_.sub_camera_info_best_effort_not_reliable_ ?
                             rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                             rclcpp::QoS{rclcpp::ServicesQoS()};
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cxt_.camera_info_sub_topic_,
        camera_info_qos,
        [this](const sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void
        {
          if (!camera_info_) {
            camera_info_ = std::make_unique<CameraInfo>(*msg);
            // Save the info message because we pass it along with the observations.
            camera_info_msg_ = std::make_unique<sensor_msgs::msg::CameraInfo>(*msg);
          }
        });

      auto image_raw_qos = cxt_.sub_image_raw_best_effort_not_reliable_ ?
                             rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                             rclcpp::QoS{rclcpp::ServicesQoS()};
      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        cxt_.image_raw_sub_topic_,
        image_raw_qos,
        [this](sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
          // the stamp to use for all published messages derived from this image message.
          auto stamp{msg->header.stamp};

          if (!camera_info_) {
            RCLCPP_DEBUG(get_logger(), "Ignore image message because no camera_info has been received yet.");

          } else if ((stamp.nanosec == 0l && stamp.sec == 0l) || stamp == last_image_stamp_) {
            RCLCPP_DEBUG(get_logger(), "Ignore image message because stamp is zero or the same as the previous.");

          } else {
            // rviz doesn't like it when time goes backward when a bag is played again.
            // The stamp_msgs_with_current_time_ parameter can help this by replacing the
            // image message time with the current time.
            stamp = cxt_.stamp_msgs_with_current_time_ ? builtin_interfaces::msg::Time(now()) : stamp;
            process_image(std::move(msg), stamp);
          }

          last_image_stamp_ = stamp;
        });

      map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        cxt_.fiducial_map_sub_topic_,
        16,
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          map_ = std::make_unique<Map>(*msg);
        });

      (void) camera_info_sub_;
      (void) image_raw_sub_;
      (void) map_sub_;
      RCLCPP_INFO(get_logger(), "vloc_node ready");
    }

  private:
    void process_image(sensor_msgs::msg::Image::UniquePtr image_msg, std_msgs::msg::Header::_stamp_type stamp)
    {
      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr gray{cv_bridge::toCvCopy(*image_msg, "mono8")};

      // If we are going to publish an annotated image, make a copy of
      // the original message image. If no annotated image is to be published,
      // then just make an empty image pointer. The routines need to check
      // that the pointer is valid before drawing into it.
      cv_bridge::CvImagePtr color_marked;
      cv::Mat mat_with_msg_data;
      if (cxt_.publish_image_marked_ &&
          count_subscribers(cxt_.image_marked_pub_topic_) > 0) {
        // The toCvShare only makes ConstCvImage because they don't want
        // to modify the original message data. I want to modify the original
        // data so I create another CvImage that is not const and steal the
        // image data.
        std::shared_ptr<void const> tracked_object;
        auto const_color_marked = cv_bridge::toCvShare(*image_msg, tracked_object);
        mat_with_msg_data = const_color_marked->image; // opencv does not copy the image data on assignment
        color_marked = cv_bridge::CvImagePtr{
          new cv_bridge::CvImage{const_color_marked->header,
                                 const_color_marked->encoding,
                                 mat_with_msg_data}};
      }

      FiducialMath fm(*camera_info_);

      // Detect the markers in this image and create a list of
      // observations.
      auto observations = fm.detect_markers(gray, color_marked, cxt_.corner_refinement_method_);

      // If there is a map, find t_map_marker for each detected
      // marker. The t_map_markers has an entry for each element
      // in observations. If the marker wasn't found in the map, then
      // the t_map_marker entry has is_valid() as false.
      // Debugging hint: If the markers in color_marked are not outlined
      // in green, then they haven't been detected. If the markers in
      // color_marked are outlined but they have no axes drawn, then vmap_node
      // is not running or has not been able to find the starting node.
      if (map_) {
        auto t_map_markers = map_->find_t_map_markers(observations);

        TransformWithCovariance t_map_camera;

        // Only try to determine the location if markers were detected.
        if (observations.size()) {

//        RCLCPP_INFO(get_logger(), "%i observations", observations.size());
//        for (auto &obs : observations.observations()) {
//          RCLCPP_INFO(get_logger(),
//                      " Marker %i, p0[%8.3f, %8.3f], p1[%8.3f, %8.3f], p2[%8.3f, %8.3f], p3[%8.3f, %8.3f]",
//                      obs.id(),
//                      obs.x0(), obs.y0(), obs.x1(), obs.y1(),
//                      obs.x2(), obs.y2(), obs.x3(), obs.y3()
//          );
//        }

          // Find the camera pose from the observations.
          t_map_camera = localizer_.simultaneous_t_map_camera(observations, t_map_markers, color_marked, fm);

          if (t_map_camera.is_valid()) {

            TransformWithCovariance t_map_base{t_map_camera.transform() * cxt_.t_camera_base_.transform()};

            // Publish the camera an/or base pose in the map frame
            if (cxt_.publish_camera_pose_) {
              auto pose_msg = to_PoseWithCovarianceStamped_msg(t_map_camera, stamp, cxt_.map_frame_id_);
              // add some fixed variance for now.
              add_fixed_covariance(pose_msg.pose);
              camera_pose_pub_->publish(pose_msg);
            }
            if (cxt_.publish_base_pose_) {
              auto pose_msg = to_PoseWithCovarianceStamped_msg(t_map_base, stamp, cxt_.map_frame_id_);
              // add some fixed variance for now.
              add_fixed_covariance(pose_msg.pose);
              base_pose_pub_->publish(pose_msg);
            }

            // Publish odometry of the camera and/or the base.
            if (cxt_.publish_camera_odom_) {
              auto odom_msg = to_odom_message(stamp, cxt_.camera_frame_id_, t_map_camera);
              add_fixed_covariance(odom_msg.pose);
              camera_odometry_pub_->publish(odom_msg);
            }
            if (cxt_.publish_base_odom_) {
              auto odom_msg = to_odom_message(stamp, cxt_.base_frame_id_, t_map_base);
              add_fixed_covariance(odom_msg.pose);
              base_odometry_pub_->publish(odom_msg);
            }

            // Also publish the camera's tf
            if (cxt_.publish_tfs_) {
              auto tf_message = to_tf_message(stamp, t_map_camera, t_map_base);
              tf_message_pub_->publish(tf_message);
            }

            // if requested, publish the camera tf as determined from each marker.
            if (cxt_.publish_tfs_per_marker_) {
              auto t_map_cameras = localizer_.markers_t_map_cameras(observations, t_map_markers, fm);
              auto tf_message = to_markers_tf_message(stamp, observations, t_map_cameras);
              if (!tf_message.transforms.empty()) {
                tf_message_pub_->publish(tf_message);
              }
            }

            // Publish the observations
            auto observations_msg = observations.to_msg(stamp, image_msg->header.frame_id, *camera_info_msg_);
            observations_pub_->publish(observations_msg);
          }
        }
      }

      // Publish an annotated image if requested. Even if there is no map.
      if (color_marked) {
        // The marking has been happening on the original message.
        // Republish it now.
        if (cxt_.stamp_msgs_with_current_time_) {
          image_msg->header.stamp = now();
        }
        image_marked_pub_->publish(std::move(image_msg));
      }
    }

    nav_msgs::msg::Odometry to_odom_message(std_msgs::msg::Header::_stamp_type stamp,
                                            const std::string &child_frame_id,
                                            const TransformWithCovariance &t)
    {
      nav_msgs::msg::Odometry odom_message;

      odom_message.header.stamp = stamp;
      odom_message.header.frame_id = cxt_.map_frame_id_;
      odom_message.child_frame_id = child_frame_id;
      odom_message.pose = to_PoseWithCovariance_msg(t);
      return odom_message;
    }

    tf2_msgs::msg::TFMessage to_tf_message(std_msgs::msg::Header::_stamp_type stamp,
                                           const TransformWithCovariance &t_map_camera,
                                           const TransformWithCovariance &t_map_base)
    {
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cxt_.map_frame_id_;

      // The camera_frame_id parameter is non-empty to publish the camera tf.
      // The base_frame_id parameter is non-empty to publish the base tf.
      if (!cxt_.camera_frame_id_.empty()) {
        msg.child_frame_id = cxt_.camera_frame_id_;
        msg.transform = tf2::toMsg(t_map_camera.transform());
        tf_message.transforms.emplace_back(msg);
      }
      if (!cxt_.base_frame_id_.empty()) {
        msg.child_frame_id = cxt_.base_frame_id_;
        msg.transform = tf2::toMsg(t_map_base.transform());
        tf_message.transforms.emplace_back(msg);
      }

      return tf_message;
    }

    tf2_msgs::msg::TFMessage to_markers_tf_message(
      std_msgs::msg::Header::_stamp_type stamp,
      const Observations &observations,
      const std::vector<TransformWithCovariance> &t_map_cameras)
    {
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cxt_.map_frame_id_;

      for (int i = 0; i < observations.size(); i += 1) {
        auto &observation = observations.observations()[i];
        auto &t_map_camera = t_map_cameras[i];

        if (t_map_camera.is_valid()) {

          if (!cxt_.camera_frame_id_.empty()) {
            std::ostringstream oss_child_frame_id;
            oss_child_frame_id << cxt_.camera_frame_id_ << "_m" << std::setfill('0') << std::setw(3)
                               << observation.id();
            msg.child_frame_id = oss_child_frame_id.str();
            msg.transform = tf2::toMsg(t_map_camera.transform());
            tf_message.transforms.emplace_back(msg);
          }
        }
      }

      return tf_message;
    }

    void add_fixed_covariance(geometry_msgs::msg::PoseWithCovariance &pwc)
    {
      // A hack for now.
      pwc.covariance[0] = 6e-3;
      pwc.covariance[7] = 6e-3;
      pwc.covariance[14] = 6e-3;
      pwc.covariance[21] = 2e-3;
      pwc.covariance[28] = 2e-3;
      pwc.covariance[35] = 2e-3;
    }
  };

  std::shared_ptr<rclcpp::Node> vloc_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VlocNode(options));
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VlocNode)
