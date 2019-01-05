
#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/map.hpp"
#include "flock_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"

#include "eigen_util.hpp"
#include "map.hpp"

namespace flock_vlam {

  class VlocNode : public rclcpp::Node
  {
  private:
    Map map_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<flock_vlam_msgs::msg::Map>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
    rclcpp::Publisher<flock_vlam_msgs::msg::Observations>::SharedPtr observations_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_;

    bool have_camera_info_{false};
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    float marker_length_ {0.18};

  public:

    explicit VlocNode()
    : Node("vloc_node"), map_(*this)
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
    }

  private:

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      if (!have_camera_info_) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, 0.);
        camera_matrix_.at<double>(0, 0) = msg->k[0];
        camera_matrix_.at<double>(0, 2) = msg->k[2];
        camera_matrix_.at<double>(1, 1) = msg->k[4];
        camera_matrix_.at<double>(1, 2) = msg->k[5];
        camera_matrix_.at<double>(2, 2) = 1.;

        // ROS and OpenCV (and everybody?) agree on this ordering: k1, k2, t1 (p1), t2 (p2), k3
        dist_coeffs_ = cv::Mat(1, 5, CV_64F);
        dist_coeffs_.at<double>(0) = msg->d[0];
        dist_coeffs_.at<double>(1) = msg->d[1];
        dist_coeffs_.at<double>(2) = msg->d[2];
        dist_coeffs_.at<double>(3) = msg->d[3];
        dist_coeffs_.at<double>(4) = msg->d[4];

        RCLCPP_INFO(get_logger(), "have camera info");
        have_camera_info_ = true;
      }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(msg);

      process_image(color, msg->header);
    }

    void map_callback(const flock_vlam_msgs::msg::Map::SharedPtr msg)
    {
      map_.load_from_msg(msg);
    }

    void process_image(cv_bridge::CvImagePtr color, std_msgs::msg::Header & header_msg)
    {
      // Color to gray for detection
      cv::Mat gray;
      cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

      // Stop if no markers were detected
      if (ids.size() == 0) {
        return;
      }

      // Calculate the pose of this camera in the map frame.
      Observations observations(ids, corners);
      auto t_map_camera = map_.estimate_t_map_camera(observations, marker_length_, camera_matrix_, dist_coeffs_);

      // Publish the camera pose in the map frame
      auto t_map_camera_msg = t_map_camera.to_msg(header_msg);
      if (t_map_camera.is_valid()) {
        camera_pose_pub_->publish(t_map_camera_msg);
      }

      // Publish the observations only if multiple markers exist in the image
      if (ids.size() > 1) {
        auto observations_msg = observations.to_msg(t_map_camera_msg);
        observations_pub_->publish(observations_msg);
      }
    }
  };

} // namespace detect_markers

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
