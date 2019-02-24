
#include "rclcpp/rclcpp.hpp"

#include "fiducial_math.hpp"
#include "map.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp" // todo: remove


namespace fiducial_vlam
{

//=============
// VlocNode class
//=============

  class VlocNode : public rclcpp::Node
  {
    Map map_;
    CameraInfo camera_info_;
    Localizer localizer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("camera_pose", 16);
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_pub_ =
      create_publisher<fiducial_vlam_msgs::msg::Observations>("/fiducial_observations", 16);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_ =
      create_publisher<sensor_msgs::msg::Image>("image_marked", 1);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;

    sensor_msgs::msg::CameraInfo camera_info_msg_;
    cv::Mat camera_matrix_; // todo: remove
    cv::Mat dist_coeffs_; // todo: remove

    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParameters_ = cv::aruco::DetectorParameters::create();

  public:

    VlocNode()
      : Node("vloc_node"), map_(*this), camera_info_(), localizer_(*this, map_)
    {
      detectorParameters_->doCornerRefinement = true;

      // ROS subscriptions
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
        [this](const sensor_msgs::msg::CameraInfo::UniquePtr msg)
        {
          if (!camera_info_.is_valid()) {
            // Save the info message because we pass it along with the observations.
            camera_info_msg_ = *msg;
            to_camera_info(*msg, camera_matrix_, dist_coeffs_);
            camera_info_ = CameraInfo(*msg);

            RCLCPP_INFO(this->get_logger(), "have camera info");
          }
        },
        16);

      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        [this](const sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
          if (this->camera_info_.is_valid()) {
            this->process_image(*msg);
          }
        },
        16);

      map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        "/fiducial_map",
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          this->map_.load_from_msg(*msg);
        },
        16);

      RCLCPP_INFO(get_logger(), "vloc_node ready");
    }

  private:
    void process_image(const sensor_msgs::msg::Image &image_msg)
    {
      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(image_msg);

      // Color to gray for detection
      cv::Mat gray;
      cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detectorParameters_);

      RCLCPP_DEBUG(get_logger(), "process_image: Found %d markers", ids.size());

      // Stop if no markers were detected
      if (ids.size() == 0) {
        return;
      }

      // Calculate the pose of this camera in the map frame.
      Observations observations(ids, corners);
      auto camera_pose_f_map = localizer_.average_camera_pose_f_map(observations, camera_matrix_, dist_coeffs_);

      if (camera_pose_f_map.is_valid()) {
        // Publish the camera pose in the map frame
        auto camera_pose_f_map_msg = to_PoseWithCovarianceStamped_msg(camera_pose_f_map, image_msg.header);

        // for now just publish a pose message not a pose
        geometry_msgs::msg::PoseWithCovarianceStamped cam_pose_f_map;
        cam_pose_f_map.pose.pose = camera_pose_f_map_msg.pose.pose;
        cam_pose_f_map.header = image_msg.header;
        cam_pose_f_map.header.frame_id = "map";
        cam_pose_f_map.pose.covariance[0] = 6e-3;
        cam_pose_f_map.pose.covariance[7] = 6e-3;
        cam_pose_f_map.pose.covariance[14] = 6e-3;
        cam_pose_f_map.pose.covariance[21] = 2e-3;
        cam_pose_f_map.pose.covariance[28] = 2e-3;
        cam_pose_f_map.pose.covariance[35] = 2e-3;
        camera_pose_pub_->publish(cam_pose_f_map);
      }

      // Publish the observations only if multiple markers exist in the image
      if (ids.size() > 1) {
        auto observations_msg = observations.to_msg(image_msg.header, camera_info_msg_);
        observations_pub_->publish(observations_msg);
      }

      // Publish an annotated image
      if (count_subscribers(image_marked_pub_->get_topic_name()) > 0) {

        // Compute marker poses in two ways to verify that the math is working.
        // Compute marker poses using OpenCV methods. The estimatePoseSingleMarkers() method
        // returns the pose of the marker in the camera frame - t_camera_marker.
//        std::vector<cv::Vec3d> rvecs, tvecs;
//        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // Compute marker poses using vlam info. Note this can only be done if
        // a camera pose in map frame is determined and we have a marker pose in
        // the map frame. The calculation is to take the marker location in the map
        // frame t_map_marker and transform (pre-multiply) it by t_map_camera.inverse()
        // to get t_camera_marker.
        std::vector<cv::Vec3d> rvecs_map, tvecs_map;
        localizer_.markers_pose_f_camera(camera_pose_f_map, ids, rvecs_map, tvecs_map);

        // Draw poses
//        for (int i = 0; i < rvecs.size(); i++) {
//          cv::aruco::drawAxis(color->image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.1);
//        }
        for (int i = 0; i < rvecs_map.size(); i++) {
          cv::aruco::drawAxis(color->image, camera_matrix_, dist_coeffs_, rvecs_map[i], tvecs_map[i], 0.1);
        }

        // Publish result
        image_marked_pub_->publish(color->toImageMsg());
      }
    };
  };
}

//=============
// main()
//=============

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

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
