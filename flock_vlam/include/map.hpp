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

    TransformWithCovariance(const Eigen::Affine3d & transform, double variance)
      : transform_(transform), variance_(variance) {}

    TransformWithCovariance(const cv::Vec3d & rvec, const cv::Vec3d & tvec, double variance)
      : transform_(eigen_util::to_affine(rvec, tvec)), variance_(variance) {}

    auto is_valid() { return is_valid_; }
    auto transform() { return transform_; }
    auto variance() { return variance_; }

    geometry_msgs::msg::PoseWithCovariance to_msg();
    geometry_msgs::msg::PoseWithCovarianceStamped to_msg(std_msgs::msg::Header & header);
  };

  class Observation
  {
    // The id of the marker
    int id_;

    // The 2D coordinates of the corners in the image.
    // The corners need to be in the same order as is returned
    // from cv::aruco::detectMarkers()
    std::vector<cv::Point2f> corners_image_corner_;

  public:
    Observation() = default;

    Observation(int id, std::vector<cv::Point2f> corners_image_corner)
    : id_(id), corners_image_corner_(corners_image_corner) {}

    auto id() { return id_; }
    auto corners_image_corner() { return corners_image_corner_; }
  };

  class Observations
    {
    std::vector<Observation> observations_;

  public:
    Observations(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners);

    auto observations() { return observations_; }

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

    auto id() { return id_; }
    auto t_map_marker() { return t_map_marker_; }

    // Return the 3D coordinates of the marker corners in the map frame.
    // These corners need to be in the same order as the corners returned
    // from cv::aruco::detectMarkers()
    auto corners_map_corner(float marker_length) {
      std::vector<cv::Point3d> corners_map_corner;
      corners_map_corner.push_back(cv::Point3d(-marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_map_corner.push_back(cv::Point3d( marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_map_corner.push_back(cv::Point3d( marker_length / 2.f,-marker_length / 2.f, 0.f));
      corners_map_corner.push_back(cv::Point3d(-marker_length / 2.f,-marker_length / 2.f, 0.f));
      return corners_map_corner;
    }
  };

  class Map
  {
  private:
    const rclcpp::Node & node_;
    std::map<int, Marker> markers_;

  public:
    explicit Map(rclcpp::Node & node);

    void load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg);
    TransformWithCovariance estimate_t_map_camera(Observations & observations, float marker_length,
                                                  cv::Mat camera_matrix, cv::Mat dist_coeffs);
  };
} // namespace flock_vlam

#endif //FLOCK_VLAM_MAP_H
