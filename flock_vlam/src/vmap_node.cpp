

#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"


#include "map.hpp"

namespace flock_vlam
{

//=============
// Mapper class
//=============

  class Mapper
  {
    rclcpp::Node &node_;
    Map &map_;

  public:
    Mapper(rclcpp::Node &node, Map &map)
      : node_(node), map_(map)
    {
    }

    virtual ~Mapper()
    {}

    auto &node()
    { return node_; }

    auto &map()
    { return map_; }

    virtual bool
    update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations, float marker_length,
               const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const std_msgs::msg::Header &header_msg) = 0;
  };

//=============
// MapperSimpleAverage class
//=============

  class MapperSimpleAverage : public Mapper
  {

  public:
    MapperSimpleAverage(rclcpp::Node &node, Map &map)
      : Mapper(node, map)
    {
    }

    virtual ~MapperSimpleAverage()
    {}

    virtual bool
    update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations, float marker_length,
               const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const std_msgs::msg::Header &header_msg)
    {
      return false;
    }
  };

//=============
// VmapNode class
//=============

  class VmapNode : public rclcpp::Node
  {
    Map map_;
    Localizer localizer_;
    std::shared_ptr<Mapper> mapper_;

    float marker_length_{0.18415};

  public:

    explicit VmapNode()
      : Node("vmap_node"), map_(*this), localizer_(*this, map_)
    {
      mapper_.reset(new MapperSimpleAverage(*this, map_));

      // ROS subscriptions
      auto observations_sub_cb = std::bind(&VmapNode::observations_callback, this, std::placeholders::_1);
      observations_sub_ = create_subscription<flock_vlam_msgs::msg::Observations>("/flock_observations",
                                                                                  observations_sub_cb);

      // ROS publishers
      map_pub_ = create_publisher<flock_vlam_msgs::msg::Map>("/flock_map", 128);

    }

  private:

    void observations_callback(const flock_vlam_msgs::msg::Observations::SharedPtr msg)
    {

      // Get the cameraInfo from the message
      cv::Mat camera_matrix;
      cv::Mat dist_coeffs;
      tf2_util::load_camera_info(msg->camera_info, camera_matrix, dist_coeffs);

      // Get observations from the message.
      Observations observations(*msg);

      // Estimate the camera pose using the latest map estimate
      auto camera_pose_f_map = localizer_.estimate_camera_pose_f_map(observations, marker_length_, camera_matrix,
                                                                     dist_coeffs);

      // Update our map with the observations
      auto doPub = mapper_->update_map(camera_pose_f_map, observations, marker_length_,
                                       camera_matrix, dist_coeffs, msg->header);

      // Publish the new map if requested
      if (doPub) {
        auto map_msg = map_.to_map_msg();
      }
    }

    rclcpp::Subscription<flock_vlam_msgs::msg::Observations>::SharedPtr observations_sub_;

    rclcpp::Publisher<flock_vlam_msgs::msg::Map>::SharedPtr map_pub_;
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
  auto node = std::make_shared<flock_vlam::VmapNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
