
#ifndef FLOCK_VLAM_TF_UTIL_HPP
#define FLOCK_VLAM_TF_UTIL_HPP

#include "opencv2/core.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/LinearMath/Transform.h"

namespace tf2_util
{
  tf2::Transform to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

  void to_cv_rvec_tvec(const tf2::Transform &transform, cv::Vec3d &rvec, cv::Vec3d &tvec);

  void load_camera_info(const sensor_msgs::msg::CameraInfo &msg, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
}

#endif //FLOCK_VLAM_TF_UTIL_HPP
