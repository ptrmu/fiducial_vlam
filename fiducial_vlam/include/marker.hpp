#ifndef FIDUCIAL_VLAM_MARKER_HPP
#define FIDUCIAL_VLAM_MARKER_HPP

#include "transform_with_covariance.hpp"

#include "opencv2/calib3d.hpp" // remove this when finished refactoring

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

    auto &marker_pose_f_map() const
    { return t_map_marker_; }

    auto &t_map_marker() const
    { return t_map_marker_; }

    // Return the 3D coordinate cv::Point3d vectors of the marker corners in the map frame.
    // These corners need to be in the same order as the corners returned
    // from cv::aruco::detectMarkers()
    std::vector <cv::Point3d> corners_f_map(float marker_length);

    static std::vector <cv::Point3d> corners_f_marker(float marker_length);

    void update_simple_average(TransformWithCovariance &newVal)
    {
      if (!is_fixed_) {
        t_map_marker_.update_simple_average(newVal, update_count_);
        update_count_ += 1;
      }
    }
  };
}
#endif //FIDUCIAL_VLAM_MARKER_HPP
