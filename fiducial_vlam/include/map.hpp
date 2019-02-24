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

#include "convert_util.hpp"
#include "marker.hpp"
#include "observation.hpp"
#include "transform_with_covariance.hpp"

// coordinate frame conventions
//  t_destination_source is a transformation from source frame to destination frame
//  xxx_f_destination means xxx is expressed in destination frame

namespace fiducial_vlam
{
//=============
// Map class
//=============

  class Map
  {
    const rclcpp::Node &node_;
    std::map<int, Marker> markers_;
    double marker_length_;

  public:
    explicit Map(rclcpp::Node &node);

    auto &markers()
    { return markers_; }

    auto marker_length()
    { return marker_length_; }

    void load_from_msg(const fiducial_vlam_msgs::msg::Map::SharedPtr msg);

    fiducial_vlam_msgs::msg::Map to_map_msg(const std_msgs::msg::Header &header_msg, double marker_length);

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
