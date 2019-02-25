
#ifndef FIDUCIAL_VLAM_TF_UTIL_HPP
#define FIDUCIAL_VLAM_TF_UTIL_HPP

#include "transform_with_covariance.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace fiducial_vlam
{
  geometry_msgs::msg::Pose to_Pose_msg(const TransformWithCovariance &twc);

  geometry_msgs::msg::PoseWithCovariance to_PoseWithCovariance_msg(const TransformWithCovariance &twc);

  geometry_msgs::msg::PoseWithCovarianceStamped to_PoseWithCovarianceStamped_msg(
    const TransformWithCovariance &twc, const std_msgs::msg::Header &header);

  TransformWithCovariance to_TransformWithCovariance(const geometry_msgs::msg::PoseWithCovariance &pwc);
}
#endif //FIDUCIAL_VLAM_TF_UTIL_HPP
