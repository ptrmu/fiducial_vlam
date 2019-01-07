#ifndef EIGEN_UTIL_HPP
#define EIGEN_UTIL_HPP

#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include "opencv2/core.hpp"

namespace eigen_util {

  Eigen::Affine3d to_affine(const cv::Vec3d &rvec, const cv::Vec3d &tvec);
  Eigen::Affine3d to_affine(const geometry_msgs::msg::Pose &pose);

  Eigen::Vector3d to_euler(const Eigen::Affine3d &transform);
  Eigen::Vector3d to_euler(const geometry_msgs::msg::Pose pose);

  cv::Vec3d to_cv_Vec3d(const Eigen::Vector3d v);
  cv::Point3d to_cv_Point3d(const Eigen::Vector3d v);
  void to_cv_rvec_tvec(const Eigen::Affine3d &transform, cv::Vec3d &rvec, cv::Vec3d &tvec);

  geometry_msgs::msg::Pose to_pose(const Eigen::Affine3d &transform);
  geometry_msgs::msg::Pose to_pose(double tx, double ty, double tz, double qx, double qy, double qz, double qw);

  geometry_msgs::msg::PoseWithCovariance to_pose_with_covariance(const Eigen::Affine3d &transform); // Todo: add covariance
  geometry_msgs::msg::PoseWithCovariance to_pose_with_covariance(double tx, double ty, double tz, double qx, double qy, double qz, double qw);

  geometry_msgs::msg::Transform to_tf(const Eigen::Affine3d &transform);
  geometry_msgs::msg::Transform to_tf(const geometry_msgs::msg::Pose pose);

  // For testing
  bool all_close(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2, double e = 0.001);
  bool is_nan(const geometry_msgs::msg::Pose p1);
  std::ostream& operator<<(std::ostream &os, const Eigen::Affine3d &affine3d);

} // namespace eigen_util
#endif