#ifndef FLOCK_VLAM_MAP_H
#define FLOCK_VLAM_MAP_H

#include <map>

#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/map.hpp"
#include "flock_vlam_msgs/msg/observation.hpp"
#include "flock_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Transform.h"

#include "tf2_util.hpp"

// coordinate frame conventions
//  t_destination_source is a transform
//  xxx_f_destination means xxx is expressed in destination frame
//  xxx_pose_f_destination is equivalent to t_destination_xxx

namespace flock_vlam
{

  class TransformWithCovariance
  {
    bool is_valid_ {false};
    tf2::Transform transform_;
    double variance_; // add more dimensions to this at some point

  public:
    TransformWithCovariance() = default;

    TransformWithCovariance(const tf2::Transform & transform, double variance)
      : is_valid_(true), transform_(transform), variance_(variance) {}

    TransformWithCovariance(const cv::Vec3d & rvec, const cv::Vec3d & tvec, double variance)
      : is_valid_(true), transform_(tf2_util::to_tf2_transform(rvec, tvec)), variance_(variance) {}

    auto is_valid() const { return is_valid_; }
    auto transform() const { return transform_; }
    auto variance() const { return variance_; }

    geometry_msgs::msg::PoseWithCovariance to_msg();
    geometry_msgs::msg::PoseWithCovarianceStamped to_msg(std_msgs::msg::Header & header);
  };

  class Observation
  {
    // The id of the marker
    int id_;

    // The 2D pixel coordinates of the corners in the image.
    // The corners need to be in the same order as is returned
    // from cv::aruco::detectMarkers()
    std::vector<cv::Point2f> corners_f_image_;

  public:
    Observation() = default;

    Observation(int id, std::vector<cv::Point2f> corners_image_corner)
    : id_(id), corners_f_image_(corners_image_corner) {}
    explicit Observation(const flock_vlam_msgs::msg::Observation &msg)
    : id_(msg.id), corners_f_image_ {
      cv::Point2f(msg.x0, msg.y0),
      cv::Point2f(msg.x1, msg.y1),
      cv::Point2f(msg.x2, msg.y2),
      cv::Point2f(msg.x3, msg.y3)
      }
    {}

    auto id() const { return id_; }
    auto corners_f_image() const { return corners_f_image_; }
  };

  class Observations
    {
    std::vector<Observation> observations_;

  public:
    Observations(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners);
    explicit Observations(const flock_vlam_msgs::msg::Observations &msg);

    auto observations() { return observations_; }

    flock_vlam_msgs::msg::Observations to_msg(const std_msgs::msg::Header &header_msg,
                                              const sensor_msgs::msg::CameraInfo &camera_info_msg);
  };

  class Marker
  {
    // The id of the marker
    int id_;

    // The pose of the marker in the map frame
    TransformWithCovariance marker_pose_f_map_;

    // Ids of other markers seen at the same time as this marker
    std::map<int, int> links;

  public:
    Marker() = default;

    Marker(int id, TransformWithCovariance marker_pose_f_map)
      : id_(id), marker_pose_f_map_(marker_pose_f_map) {}

    auto id() const { return id_; }
    auto marker_pose_f_map() const { return marker_pose_f_map_; }
    auto t_map_marker() const { return marker_pose_f_map_; }

    // Return the 3D coordinate cv::Point3d vectors of the marker corners in the map frame.
    // These corners need to be in the same order as the corners returned
    // from cv::aruco::detectMarkers()
    std::vector<cv::Point3d> corners_f_map(float marker_length);
  };

  class Map
  {
    const rclcpp::Node & node_;
    std::map<int, Marker> markers_;

  public:
    explicit Map(rclcpp::Node & node);

    void load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg);
    TransformWithCovariance estimate_camera_pose_f_map(Observations &observations, float marker_length,
                                                       const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);
    void markers_pose_f_camera(const TransformWithCovariance &camera_pose_f_map, const std::vector<int> &ids,
                               std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs);
    void markers_pose_f_camera_tf2(Observations &observations, float marker_length,
                                   const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                                   std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs);

    bool update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations, float marker_length,
                    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const std_msgs::msg::Header &header_msg);
    flock_vlam_msgs::msg::Observations to_map_msg();

    void load_camera_info(const sensor_msgs::msg::CameraInfo &msg, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
  };
} // namespace flock_vlam

#endif //FLOCK_VLAM_MAP_H
