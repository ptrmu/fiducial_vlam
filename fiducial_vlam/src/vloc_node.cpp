
#include "rclcpp/rclcpp.hpp"

#include "fiducial_math.hpp"
#include "map.hpp"
#include "vloc_context.hpp"

#include "cv_bridge/cv_bridge.h"
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

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("camera_pose", 16);
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_pub_ =
      create_publisher<fiducial_vlam_msgs::msg::Observations>("/fiducial_observations", 16);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_ =
      create_publisher<sensor_msgs::msg::Image>("image_marked", 16);
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_ =
      create_publisher<tf2_msgs::msg::TFMessage>("tf", 16);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;


  public:
    VlocNode()
      : Node("vloc_node")
    {
      // Get parameters from the command line
      cxt_.load_parameters(*this);

      // ROS subscriptions
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
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
        "image_raw",
        [this](const sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
          // A map and cameraInfo must be received before processing can start.
          if (this->camera_info_ && this->map_) {
            this->process_image(*msg);
          }
        },
        16);

      map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        "/fiducial_map",
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

      FiducialMath fm(*camera_info_);

      // Detect the markers in this image and create a list of
      // observations.
      Observations observations(fm.detect_markers(color));

      // Stop if no markers were detected
      if (observations.size() == 0) {
        return;
      }

      // Find the camera pose from the observations.
      auto t_map_camera = localizer_.average_t_map_camera(observations, fm);

      if (t_map_camera.is_valid()) {
        // Publish the camera pose in the map frame
        auto t_map_camera_msg = to_PoseWithCovarianceStamped_msg(t_map_camera, image_msg.header);

        // for now just publish a pose message not a pose
        geometry_msgs::msg::PoseWithCovarianceStamped cam_pose_f_map;
        cam_pose_f_map.pose.pose = t_map_camera_msg.pose.pose;
        cam_pose_f_map.header = image_msg.header;
        cam_pose_f_map.header.frame_id = "map";
        cam_pose_f_map.pose.covariance[0] = 6e-3;
        cam_pose_f_map.pose.covariance[7] = 6e-3;
        cam_pose_f_map.pose.covariance[14] = 6e-3;
        cam_pose_f_map.pose.covariance[21] = 2e-3;
        cam_pose_f_map.pose.covariance[28] = 2e-3;
        cam_pose_f_map.pose.covariance[35] = 2e-3;
        camera_pose_pub_->publish(cam_pose_f_map);

        // Also publish the camera's tf
        // todo: give the tf a name based on the camera id.
        if (cxt_.publish_camera_tf_) {
          publish_camera_tf(t_map_camera);
        }
      }

      // Publish the observations
      auto observations_msg = observations.to_msg(image_msg.header, *camera_info_msg_);
      observations_pub_->publish(observations_msg);

      // Publish an annotated image
      if (count_subscribers(image_marked_pub_->get_topic_name()) > 0) {

        // The image can be annotated only if the camera pose is known.
        // but the unannotated image should still be published.
        if (t_map_camera.is_valid()) {

          // Cache a transform.
          auto tf_t_camera_map = t_map_camera.transform().inverse();

          // Loop through the ids of the markers visible in this image
          for (auto &obs : observations.observations()) {

            // Find this marker in the map
            auto marker_ptr = map_->find_marker(obs.id());
            if (marker_ptr) {
              auto &tf_t_map_marker = marker_ptr->t_map_marker().transform();

              // Found a marker that is in the map and in the image. Calculate its
              // transform to the camera frame and annotate the image.
              auto t_camera_marker = TransformWithCovariance(tf_t_camera_map * tf_t_map_marker);
              fm.annotate_image_with_marker_axis(color, t_camera_marker);
            }
          }
        }

        // Publish annotated image
        image_marked_pub_->publish(color->toImageMsg());
      }
    }

    void publish_camera_tf(const TransformWithCovariance &t_map_camera)
    {
      auto stamp = now();
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "map";
      msg.child_frame_id = "camera";
      msg.transform = tf2::toMsg(t_map_camera.transform());
      tf_message.transforms.emplace_back(msg);

      tf_message_pub_->publish(tf_message);
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
