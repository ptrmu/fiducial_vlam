
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
    VlocContext cxt_{};
    std::unique_ptr<Map> map_{};
    std::unique_ptr<CameraInfo> camera_info_{};
    std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_{};
    Localizer localizer_{map_};

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
    VlocNode()
      : Node("vloc_node")
    {
      // Get parameters from the command line
      cxt_.load_parameters(*this);

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
      if (cxt_.publish_tfs_) {
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
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cxt_.camera_info_sub_topic_,
        [this](const sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void
        {
          if (!camera_info_) {
            camera_info_ = std::make_unique<CameraInfo>(*msg);
            // Save the info message because we pass it along with the observations.
            camera_info_msg_ = std::make_unique<sensor_msgs::msg::CameraInfo>(*msg);
          }
        },
        16);

      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        cxt_.image_raw_sub_topic_,
        [this](const sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
          // A map and cameraInfo must be received before processing can start.
          if (this->camera_info_ && this->map_) {
            this->process_image(*msg);
          }
        },
        16);

      map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        cxt_.fiducial_map_sub_topic_,
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          this->map_ = std::make_unique<Map>(*msg);
        },
        16);

      RCLCPP_INFO(get_logger(), "vloc_node ready");
    }

  private:
    void process_image(const sensor_msgs::msg::Image &image_msg)
    {
      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(image_msg);

      // the stamp to use for all published messages from this image.
      auto stamp = cxt_.stamp_msgs_with_current_time_
                   ? static_cast<std_msgs::msg::Header::_stamp_type>(now())
                   : image_msg.header.stamp;

      // If we are going to publish an annotated image, make a copy of
      // the pointer to color. If no annotated image is to be published,
      // then just make an empty image pointer. The routines need to check
      // that the pointer is valid before drawing into it.
      cv_bridge::CvImagePtr color_marked;
      if (cxt_.publish_image_marked_) {
        color_marked = color;
      }

      FiducialMath fm(*camera_info_);

      // Detect the markers in this image and create a list of
      // observations and then find t_map_marker for each detected
      // marker. The t_map_markers has an entry for each element
      // in observations. If the marker wasn't found in the map, then
      // the t_map_marker entry has is_valid() as false.
      auto observations = fm.detect_markers(color, color_marked);
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
            auto odom_msg = to_odom_message(stamp, t_map_camera);
            add_fixed_covariance(odom_msg.pose);
            camera_odometry_pub_->publish(odom_msg);
          }
          if (cxt_.publish_base_odom_) {
            auto odom_msg = to_odom_message(stamp, t_map_base);
            add_fixed_covariance(odom_msg.pose);
            base_odometry_pub_->publish(odom_msg);
          }

          // Also publish the camera's tf
          if (cxt_.publish_tfs_) {
            auto tf_message = to_tf_message(stamp, t_map_camera, t_map_base);
            tf_message_pub_->publish(tf_message);
          }

          // Publish the observations
          auto observations_msg = observations.to_msg(stamp, image_msg.header.frame_id, *camera_info_msg_);
          observations_pub_->publish(observations_msg);
        }
      }

      // Publish an annotated image if requested
      if (color_marked) {
        auto marked_image_msg{color->toImageMsg()};
        marked_image_msg->header = image_msg.header;
        image_marked_pub_->publish(marked_image_msg);
      }
    }

    nav_msgs::msg::Odometry to_odom_message(std_msgs::msg::Header::_stamp_type stamp,
                                            const TransformWithCovariance &t)
    {
      nav_msgs::msg::Odometry odom_message;

      odom_message.header.stamp = stamp;
      odom_message.header.frame_id = cxt_.map_frame_id_;
      odom_message.child_frame_id = cxt_.camera_frame_id_;
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
      if (cxt_.camera_frame_id_.size()) {
        msg.child_frame_id = cxt_.camera_frame_id_;
        msg.transform = tf2::toMsg(t_map_camera.transform());
        tf_message.transforms.emplace_back(msg);
      }
      if (cxt_.base_frame_id_.size()) {
        msg.child_frame_id = cxt_.base_frame_id_;
        msg.transform = tf2::toMsg(t_map_base.transform());
        tf_message.transforms.emplace_back(msg);
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
}

// ==============================================================================
// main()
// ==============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<fiducial_vlam::VlocNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
