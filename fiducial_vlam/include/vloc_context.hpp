#ifndef FIDUCIAL_VLAM_VLOC_CONTEXT_HPP
#define FIDUCIAL_VLAM_VLOC_CONTEXT_HPP

#include <string>

#include "context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace rclcpp
{
  class Node;
}

namespace fiducial_vlam
{
#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(               /* topic for publishing fiducial observations  */ \
  fiducial_observations_pub_topic,  \
  "/fiducial_observations", std::string) \
  CXT_ELEM(               /* topic for publishing camera pose  */ \
  camera_pose_pub_topic,  \
  "camera_pose", std::string) \
  CXT_ELEM(               /* topic for publishing base pose  */ \
  base_pose_pub_topic,  \
  "base_pose", std::string) \
  CXT_ELEM(               /* topic for publishing camera odometry  */ \
  camera_odometry_pub_topic,  \
  "camera_odom", std::string) \
  CXT_ELEM(               /* topic for publishing base odometry  */ \
  base_odometry_pub_topic,  \
  "base_odom", std::string) \
  CXT_ELEM(               /* topic for republishing the image with axes added to fiducial markers  */\
  image_marked_pub_topic,  \
  "image_marked", std::string) \
  \
  CXT_ELEM(               /* topic for subscription to fiducial_vlam_msgs::msg::Map  */\
  fiducial_map_sub_topic,  \
  "/fiducial_map", std::string) \
  CXT_ELEM(               /* topic for subscription to sensor_msgs::msg::CameraInfo associated with the image  */ \
  camera_info_sub_topic,  \
  "camera_info", std::string) \
  CXT_ELEM(               /* topic for subscription to sensor_msgs::msg::Image */ \
  image_raw_sub_topic,  \
  "image_raw", std::string) \
  \
  CXT_ELEM(               /* frame_id for camera pose and tf messages - normally "map"  */ \
  map_frame_id,  \
  "map", std::string) \
  CXT_ELEM(               /* frame_id for the child in the camera tf message  */\
  camera_frame_id,  \
  "camera", std::string) \
  CXT_ELEM(               /* frame_id for the child in the base_link tf message  */\
  base_frame_id,  \
  "base_link", std::string) \
  \
  CXT_ELEM(               /* non-zero => publish the pose of the camera at every frame  */ \
  publish_camera_pose,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the pose of the base at every frame  */ \
  publish_base_pose,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the tf of the camera at every frame  */ \
  publish_tfs,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the odometry of the camera at every frame  */ \
  publish_camera_odom,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the odometry of the base at every frame  */ \
  publish_base_odom,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the image_marked at every frame  */ \
  publish_image_marked,  \
  1, int) \
  CXT_ELEM(               /* non-zero => debug mode, helpful for dealing with rviz when playing bags.  */ \
  stamp_msgs_with_current_time,  \
  0, int) \
  \
  CXT_ELEM(               /* camera=>baselink transform component */ \
  t_camera_base_x,  \
  0., double) \
  CXT_ELEM(               /* camera=>baselink transform component */ \
  t_camera_base_y,  \
  0., double) \
  CXT_ELEM(               /* camera=>baselink transform component */ \
  t_camera_base_z, \
  -0.035, double) \
  CXT_ELEM(               /* camera=>baselink transform component */ \
  t_camera_base_roll, \
  TF2SIMD_HALF_PI, double) \
  CXT_ELEM(               /* camera=>baselink transform component */ \
  t_camera_base_pitch,  \
  -TF2SIMD_HALF_PI, double) \
  CXT_ELEM(               /* camera=>baselink transform component */ \
  t_camera_base_yaw, \
  0., double) \
  /* End of list */

#define CXT_MACRO_ALL_MEMBERS \
  CXT_MEMBER(             /* transform from base frame to camera frame */ \
  t_camera_base,  \
  TransformWithCovariance) \
  /* End of list */

  struct VlocContext
  {
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_FIELD_DEF(n, a)
    CXT_MACRO_ALL_PARAMS

#undef CXT_MEMBER
#define CXT_MEMBER(n, a...) CXT_MEMBER_FIELD_DEF(n, a)
    CXT_MACRO_ALL_MEMBERS

    void load_parameters(rclcpp::Node &node);

    void validate_parameters();
  };
}

#endif //FIDUCIAL_VLAM_VLOC_CONTEXT_HPP
