#ifndef FIDUCIAL_VLAM_MAP_H
#define FIDUCIAL_VLAM_MAP_H

#include <map>

#include "convert_util.hpp"
#include "fiducial_math.hpp"
#include "observation.hpp"
#include "transform_with_covariance.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"

// coordinate frame conventions
//  t_destination_source is a transformation from source frame to destination frame
//  xxx_f_destination means xxx is expressed in destination frame

namespace fiducial_vlam
{
// ==============================================================================
// Marker class
// ==============================================================================

  class Marker
  {
    // The id of the marker
    int id_;

    // The pose of the marker in the map frame
    TransformWithCovariance t_map_marker_;

    // Prevent modification if true
    bool is_fixed_{false};

    // Count of updates
    int update_count_;

  public:
    Marker() = default;

    Marker(int id, const TransformWithCovariance &t_map_marker)
      : id_(id), t_map_marker_(t_map_marker), update_count_(1)
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

    auto &t_map_marker() const
    { return t_map_marker_; }

    void set_t_map_marker(TransformWithCovariance t_map_marker)
    { t_map_marker_ = t_map_marker; }
  };

// ==============================================================================
// Map class
// ==============================================================================

  class Map
  {
    std::map<int, Marker> markers_;
    double marker_length_;

  public:
    Map();

    explicit Map(const fiducial_vlam_msgs::msg::Map &msg);

    const auto &markers() const
    { return markers_; }

    const auto marker_length() const
    { return marker_length_; }

    void set_marker_length(int marker_length)
    { marker_length_ = marker_length; }

    Marker * find_marker(int id);

    void add_marker(int id, Marker marker);

    std::unique_ptr<fiducial_vlam_msgs::msg::Map>
    to_map_msg(const std_msgs::msg::Header &header_msg, double marker_length);
  };

// ==============================================================================
// Localizer class
// ==============================================================================

  class Localizer
  {
    const std::shared_ptr<Map> &map_;

  public:
    explicit Localizer(const std::shared_ptr<Map> &map);

    TransformWithCovariance average_t_map_camera(Observations &observations, FiducialMath &fm);
  };

// ==============================================================================
// Utility
// ==============================================================================

//  void log_tf_transform(rclcpp::Node &node, std::string s, const tf2::Transform &transform);

} // namespace fiducial_vlam

#endif //FIDUCIAL_VLAM_MAP_H
