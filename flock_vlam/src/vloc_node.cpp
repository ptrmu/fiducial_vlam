
#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/map.hpp"
#include "flock_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"

#include "map.hpp"

namespace flock_vlam
{

//=============
// VlocNode class
//=============

  class VlocNode : public rclcpp::Node
  {
    Map map_;
    Localizer localizer_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<flock_vlam_msgs::msg::Map>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
    rclcpp::Publisher<flock_vlam_msgs::msg::Observations>::SharedPtr observations_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_;

    bool have_camera_info_{false};
    sensor_msgs::msg::CameraInfo cameraInfo_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParameters_ = cv::aruco::DetectorParameters::create();

    float marker_length_{0.18415};

  public:

    explicit VlocNode()
      : Node("vloc_node"), map_(*this), localizer_(*this, map_)
    {
      // ROS subscriptions
      auto cameraInfo_cb = std::bind(&VlocNode::camera_info_callback, this, std::placeholders::_1);
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", cameraInfo_cb);

      auto image_raw_sub_cb = std::bind(&VlocNode::image_callback, this, std::placeholders::_1);
      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>("image_raw", image_raw_sub_cb);

      auto map_sub_cb = std::bind(&VlocNode::map_callback, this, std::placeholders::_1);
      map_sub_ = create_subscription<flock_vlam_msgs::msg::Map>("/flock_map", map_sub_cb);


      // ROS publishers
      camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("camera_pose", 1);
      observations_pub_ = create_publisher<flock_vlam_msgs::msg::Observations>("/flock_observations", 1);
      image_marked_pub_ = create_publisher<sensor_msgs::msg::Image>("image_marked", 1);

      detectorParameters_->doCornerRefinement = true;

      RCLCPP_INFO(get_logger(), "vloc_node ready");
    }

  private:

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      if (!have_camera_info_) {
        // Save the info message because we pass it along with the observations.
        cameraInfo_ = *msg;
        tf2_util::load_camera_info(*msg, camera_matrix_, dist_coeffs_);

        RCLCPP_INFO(get_logger(), "have camera info");
        have_camera_info_ = true;
      }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      if (!have_camera_info_) {
        return;
      }

      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(msg);

      process_image(color, msg->header);
    }

    void map_callback(const flock_vlam_msgs::msg::Map::SharedPtr msg)
    {
      map_.load_from_msg(msg);
    }

    void process_image(cv_bridge::CvImagePtr color, std_msgs::msg::Header &header_msg)
    {
      // Color to gray for detection
      cv::Mat gray;
      cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detectorParameters_);

      RCLCPP_INFO(get_logger(), "process_image: Found %d markers", ids.size());

      // Stop if no markers were detected
      if (ids.size() == 0) {
        return;
      }

      // Calculate the pose of this camera in the map frame.
      Observations observations(ids, corners);
      auto camera_pose_f_map = localizer_.estimate_camera_pose_f_map(observations, marker_length_, camera_matrix_,
                                                                     dist_coeffs_);

      // Publish the camera pose in the map frame
      auto camera_pose_f_map_msg = camera_pose_f_map.to_msg(header_msg);
      if (camera_pose_f_map.is_valid()) {
        camera_pose_pub_->publish(camera_pose_f_map_msg);
      }

      // Publish the observations only if multiple markers exist in the image
      if (ids.size() > 1) {
        auto observations_msg = observations.to_msg(header_msg, cameraInfo_);
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
  auto node = std::make_shared<flock_vlam::VlocNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
