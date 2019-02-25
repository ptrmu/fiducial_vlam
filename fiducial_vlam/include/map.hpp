#ifndef FIDUCIAL_VLAM_MAP_H
#define FIDUCIAL_VLAM_MAP_H

#include <map>

#include "rclcpp/rclcpp.hpp"

#include "convert_util.hpp"
#include "fiducial_math.hpp"
#include "marker.hpp"
#include "observation.hpp"
#include "transform_with_covariance.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observation.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Transform.h"

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
    std::map<int, Marker> markers_;
    double marker_length_;

  public:
    Map();

    explicit Map(const fiducial_vlam_msgs::msg::Map &msg);

    auto &markers()
    { return markers_; }

    auto marker_length() const
    { return marker_length_; }

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
    const std::shared_ptr<Map> &map_;

  public:
    explicit Localizer(const std::shared_ptr<Map> &map);

    TransformWithCovariance average_t_map_camera(Observations &observations, FiducialMath &fm);
  };

//=============
// Utility
//=============

  void log_tf_transform(rclcpp::Node &node, std::string s, const tf2::Transform &transform);

} // namespace fiducial_vlam

#endif //FIDUCIAL_VLAM_MAP_H
