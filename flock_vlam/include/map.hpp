#ifndef FLOCK_VLAM_MAP_H
#define FLOCK_VLAM_MAP_H

#include <map>
#include <flock_vlam_msgs/msg/observations__struct.hpp>

#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "eigen_util.hpp"

namespace flock_vlam
{


  class TransformWithCovariance
  {
    bool is_valid_ {false};
    Eigen::Affine3d transform_;
    double variance_; // add more dimensions to this at some point

  public:
    TransformWithCovariance() = default;

    TransformWithCovariance(Eigen::Affine3d transform, double variance)
    : transform_(transform), variance_(variance) {}

    bool is_valid() { return is_valid_; }
    geometry_msgs::msg::PoseWithCovariance to_msg();
    geometry_msgs::msg::PoseWithCovarianceStamped to_msg(std_msgs::msg::Header & header);
  };

  class Observation
  {
    // The id of the marker
    int id_;

    // The pose of the marker in the camera frame
    TransformWithCovariance t_camera_marker_;

  public:
    Observation() = default;

    Observation(int id, TransformWithCovariance t_camera_marker)
    : id_(id), t_camera_marker_(t_camera_marker) {}
  };

  class Observations
    {
    std::vector<std::shared_ptr<Observation>> observations_;

  public:
    Observations(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners);

    flock_vlam_msgs::msg::Observations to_msg(geometry_msgs::msg::PoseWithCovarianceStamped & t_map_camera);
  };

  class Marker
  {
    // The id of the marker
    int id_;

    // The pose of the marker in the map frame
    TransformWithCovariance t_map_marker_;

    // Ids of other markers seen at the same time as this marker
    std::map<int, int> links;

  public:
    Marker() = default;

    Marker(int id, TransformWithCovariance t_map_marker)
      : id_(id), t_map_marker_(t_map_marker) {}
  };

  class Map
  {
  private:
    const rclcpp::Node & node_;
    std::map<int, Marker> markers_;

  public:
    explicit Map(rclcpp::Node & node);

    void load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg);
    TransformWithCovariance estimate_t_map_camera(Observations & observations);
  };
} // namespace flock_vlam

#endif //FLOCK_VLAM_MAP_H
