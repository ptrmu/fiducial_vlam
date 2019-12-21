

#include "convert_util.hpp"

#include "transform_with_covariance.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

namespace fiducial_vlam
{
  geometry_msgs::msg::Pose to_Pose_msg(const TransformWithCovariance &twc)
  {
    geometry_msgs::msg::Pose pose;
    toMsg(twc.transform(), pose);
    return pose;
  }

  geometry_msgs::msg::PoseWithCovariance to_PoseWithCovariance_msg(const TransformWithCovariance &twc)
  {
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.pose = to_Pose_msg(twc);
    msg.covariance = twc.cov();
    return msg;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped to_PoseWithCovarianceStamped_msg(
    const TransformWithCovariance &twc,
    std_msgs::msg::Header::_stamp_type stamp,
    std_msgs::msg::Header::_frame_id_type frame_id)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.pose = to_PoseWithCovariance_msg(twc);
    return msg;
  }

  TransformWithCovariance to_TransformWithCovariance(const geometry_msgs::msg::PoseWithCovariance &pwc)
  {
    tf2::Transform tf;
    fromMsg(pwc.pose, tf);
    return TransformWithCovariance(tf, pwc.covariance);
  }
}

