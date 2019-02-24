#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

#include "map.hpp"
#include "transform_with_covariance.hpp"

#include "sensor_msgs/msg/camera_info.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// CameraInfo class
// ==============================================================================

  class CameraInfo
  {
    class CvCameraInfo;

    std::shared_ptr<CvCameraInfo> cv_;

  public:
    CameraInfo() = delete;

    explicit CameraInfo(const sensor_msgs::msg::CameraInfo &camera_info);

    auto &cv() const
    { return cv_; }
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  class FiducialMath
  {
    const CameraInfo ci_;
    const double marker_length_;

  public:
    FiducialMath(const CameraInfo &camera_info, double marker_length)
      : ci_(camera_info), marker_length_(marker_length)
    {}

    FiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg, double marker_length)
      : ci_(camera_info_msg), marker_length_(marker_length)
    {}

    TransformWithCovariance estimate_t_map_marker(const Observation &observation,
                                                  const TransformWithCovariance &camera_pose_f_map);
  };
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
