#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

#include "observation.hpp"
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
    CameraInfo();

    explicit CameraInfo(const sensor_msgs::msg::CameraInfo &camera_info);

    auto &cv() const
    { return cv_; }

    bool is_valid() const
    { return cv_ != nullptr; }
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  class FiducialMath
  {
    class CvFiducialMath;

    std::shared_ptr<CvFiducialMath> cv_;

  public:
    FiducialMath(const CameraInfo &camera_info, double marker_length);

    FiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg, double marker_length);

    TransformWithCovariance solve_t_camera_marker(const Observation &observation);
  };
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
