
#ifndef FIDUCIAL_VLAM_TF_UTIL_HPP
#define FIDUCIAL_VLAM_TF_UTIL_HPP

#include "opencv2/core.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/LinearMath/Transform.h"

#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{
  geometry_msgs::msg::Pose to_Pose_msg(const TransformWithCovariance &twc);

  geometry_msgs::msg::PoseWithCovariance to_PoseWithCovariance_msg(const TransformWithCovariance &twc);

  geometry_msgs::msg::PoseWithCovarianceStamped to_PoseWithCovarianceStamped_msg(
    const TransformWithCovariance &twc, const std_msgs::msg::Header &header);

  TransformWithCovariance to_TransformWithCovariance(const geometry_msgs::msg::PoseWithCovariance &pwc);

  tf2::Transform to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

  void to_cv_rvec_tvec(const tf2::Transform &transform, cv::Vec3d &rvec, cv::Vec3d &tvec);

  void to_camera_info(const sensor_msgs::msg::CameraInfo &msg, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
}
#endif //FIDUCIAL_VLAM_TF_UTIL_HPP
