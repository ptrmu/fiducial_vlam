#ifndef FIDUCIAL_VLAM_VLOC_CONTEXT_HPP
#define FIDUCIAL_VLAM_VLOC_CONTEXT_HPP

#include "rclcpp/rclcpp.hpp"

#include "context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(               /* topic for publishing fiducial observations  */ \
  fiducial_observations_pub_topic,  \
  "/fiducial_observations", std::string) \
  CXT_ELEM(               /* topic for publishing camera pose  */ \
  camera_pose_pub_topic,  \
  "camera_pose", std::string) \
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
  CXT_ELEM(               /* frame_id for the child in the tf message  */\
  camera_frame_id,  \
  "camera", std::string) \
  \
  CXT_ELEM(               /* non-zero => publish the tf of the camera at every frame  */ \
  publish_tfs,  \
  1, int) \
  CXT_ELEM(               /* non-zero => publish the image_marked at every frame  */ \
  publish_image_marked,  \
  1, int) \
  CXT_ELEM(               /* non-zero => debug mode, helpful for dealing with rviz when playing bags.  */ \
  stamp_msgs_with_current_time,  \
  0, int) \
  /* End of list */

#define CXT_MACRO_ALL_MEMBERS \
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
