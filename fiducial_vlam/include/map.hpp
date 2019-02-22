#ifndef FIDUCIAL_VLAM_MAP_H
#define FIDUCIAL_VLAM_MAP_H

#include <map>

#include "rclcpp/rclcpp.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observation.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Transform.h"

#include "tf2_util.hpp"

// coordinate frame conventions
//  t_destination_source is a transform
//  xxx_f_destination means xxx is expressed in destination frame
//  xxx_pose_f_destination is equivalent to t_destination_xxx

namespace fiducial_vlam
{

//=============
// TransformWithCovariance class
//=============

  class TransformWithCovariance
  {
    bool is_valid_{false};
    tf2::Transform transform_;
    double variance_; // add more dimensions to this at some point

  public:
    TransformWithCovariance() = default;

    TransformWithCovariance(const tf2::Transform &transform, double variance)
      : is_valid_(true), transform_(transform), variance_(variance)
    {}

    TransformWithCovariance(const cv::Vec3d &rvec, const cv::Vec3d &tvec, double variance)
      : is_valid_(true), transform_(tf2_util::to_tf2_transform(rvec, tvec)), variance_(variance)
    {}

    explicit TransformWithCovariance(geometry_msgs::msg::PoseWithCovariance pose);

    auto is_valid() const
    { return is_valid_; }

    auto transform() const
    { return transform_; }

    auto variance() const
    { return variance_; }

    geometry_msgs::msg::Pose to_pose_msg();

    geometry_msgs::msg::PoseWithCovariance to_pose_with_covariance_msg();

    geometry_msgs::msg::PoseStamped to_pose_stamped_msg(std_msgs::msg::Header &header);

    geometry_msgs::msg::PoseWithCovarianceStamped to_pose_with_covariance_stamped_msg(std_msgs::msg::Header &header);

    void update_simple_average(TransformWithCovariance &newVal, int previous_update_count);
  };

//=============
// Observation class
//=============

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
      : id_(id), corners_f_image_(corners_image_corner)
    {}

    explicit Observation(const fiducial_vlam_msgs::msg::Observation &msg)
      : id_(msg.id), corners_f_image_{
      cv::Point2f(msg.x0, msg.y0),
      cv::Point2f(msg.x1, msg.y1),
      cv::Point2f(msg.x2, msg.y2),
      cv::Point2f(msg.x3, msg.y3)
    }
    {}

    auto id() const
    { return id_; }

    auto corners_f_image() const
    { return corners_f_image_; }
  };

//=============
// Observations class
//=============

  class Observations
  {
    std::vector<Observation> observations_;

  public:
    Observations(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners);

    explicit Observations(const fiducial_vlam_msgs::msg::Observations &msg);

    auto observations()
    { return observations_; }

    fiducial_vlam_msgs::msg::Observations to_msg(const std_msgs::msg::Header &header_msg,
                                              const sensor_msgs::msg::CameraInfo &camera_info_msg);
  };

//=============
// Marker class
//=============

  class Marker
  {
    // The id of the marker
    int id_;

    // The pose of the marker in the map frame
    TransformWithCovariance marker_pose_f_map_;

    // Prevent modification if true
    bool is_fixed_{false};

    // Count of updates
    int update_count_;

  public:
    Marker() = default;

    Marker(int id, TransformWithCovariance marker_pose_f_map)
      : id_(id), marker_pose_f_map_(marker_pose_f_map), update_count_(1)
    {}

    auto id() const
    { return id_; }

    auto is_fixed() const
    { return is_fixed_; }

    void set_is_fixed(bool is_fixed)
    { is_fixed_ = is_fixed; }

    auto update_count() const
    { return update_count_; }

    void set_update_count(int update_count)
    { update_count_ = update_count; }

    auto marker_pose_f_map() const
    { return marker_pose_f_map_; }

    auto t_map_marker() const
    { return marker_pose_f_map_; }

    // Return the 3D coordinate cv::Point3d vectors of the marker corners in the map frame.
    // These corners need to be in the same order as the corners returned
    // from cv::aruco::detectMarkers()
    std::vector<cv::Point3d> corners_f_map(float marker_length);

    static std::vector<cv::Point3d> corners_f_marker(float marker_length);

    void update_simple_average(TransformWithCovariance &newVal)
    {
      if (!is_fixed_) {
        marker_pose_f_map_.update_simple_average(newVal, update_count_);
        update_count_ += 1;
      }
    }
  };

//=============
// Map class
//=============

  class Map
  {
    const rclcpp::Node &node_;
    std::map<int, Marker> markers_;
    float marker_length_;

  public:
    explicit Map(rclcpp::Node &node);

    auto &markers()
    { return markers_; }

    auto marker_length()
    { return marker_length_; }

    void load_from_msg(const fiducial_vlam_msgs::msg::Map::SharedPtr msg);

    fiducial_vlam_msgs::msg::Map to_map_msg(const std_msgs::msg::Header &header_msg, float marker_length);

    void to_YAML_string(std::string &yaml);

    void from_YAML_string(std::string &yaml);

    void load_from_file(std::string full_path);

    void save_to_file(std::string full_path);
  };

//=============
// Localizer class
//=============

  class Localizer
  {
    rclcpp::Node &node_;
    Map &map_;

  public:
    Localizer(rclcpp::Node &node, Map &map);

    TransformWithCovariance average_camera_pose_f_map(Observations &observations,
                                                      const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);

    TransformWithCovariance estimate_camera_pose_f_map(Observations &observations,
                                                       const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);

    void markers_pose_f_camera(const TransformWithCovariance &camera_pose_f_map, const std::vector<int> &ids,
                               std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs);

    void markers_pose_f_camera_tf2(Observations &observations,
                                   const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                                   std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs);
  };

//=============
// Utility
//=============

  void log_tf_transform(rclcpp::Node &node, const std::string s, const tf2::Transform &transform);

} // namespace fiducial_vlam

#endif //FIDUCIAL_VLAM_MAP_H
