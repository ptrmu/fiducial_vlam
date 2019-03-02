#ifndef FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
#define FIDUCIAL_VLAM_VMAP_CONTEXT_HPP

#include "rclcpp/rclcpp.hpp"

#include "context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(               /* topic for publishing map of markers  */ \
  fiducial_map_pub_topic,  \
  "/fiducial_map", std::string) \
  CXT_ELEM(               /* topic for publishing rviz visualizations of the fiducial markers  */ \
  fiducial_markers_pub_topic,  \
  "fiducial_markers", std::string) \
  \
  CXT_ELEM(               /* topic for subscription to fiducial_vlam_msgs::msg::Observations  */ \
  fiducial_observations_sub_topic,  \
  "/fiducial_observations", std::string) \
  \
  CXT_ELEM(               /* frame_id for marker and tf messages - normally "map"  */ \
  map_frame_id,  \
  "map", std::string) \
  CXT_ELEM(               /* frame_id for the child in the marker tf message  */\
  marker_prefix_frame_id,  \
  "marker_", std::string) \
  \
  CXT_ELEM(               /* non-zero => publish the tf of all the known markers  */ \
  publish_tfs, \
  1, int) \
  CXT_ELEM(               /* non-zero => publish a shape that represents a marker  */ \
  publish_marker_visualizations, \
  1, int) \
  CXT_ELEM(               /* Hz => rate at which the marker map is published */ \
  marker_map_publish_frequency_hz, \
  0., double) \
  \
  CXT_ELEM(               /* name of the file to store the marker map in  */  \
  marker_map_save_full_filename, \
  "fiducial_marker_locations.yaml", std::string) \
  CXT_ELEM(               /* name of the file to load the marker map from  */  \
  marker_map_load_full_filename, \
  "fiducial_marker_locations_saved.yaml", std::string) \
  CXT_ELEM(               /* non-zero => create a new map  */\
  make_not_use_map,  \
  0, int) \
  CXT_ELEM(               /* 0->marker id, pose from file, 1->marker id, pose as parameter, 2->camera pose as parameter  */ \
  map_init_style, \
  1, int) \
  CXT_ELEM(               /* marker id for map initialization */ \
  map_init_id,  \
  1, int) \
  CXT_ELEM(               /* pose component for map initialization */ \
  map_init_pose_x,  \
  0., double) \
  CXT_ELEM(               /* pose component for map initialization */ \
  map_init_pose_y,  \
  0., double) \
  CXT_ELEM(               /* pose component for map initialization */ \
  map_init_pose_z, \
  1., double) \
  CXT_ELEM(               /* pose component for map initialization */ \
  map_init_pose_roll, \
  TF2SIMD_HALF_PI, double) \
  CXT_ELEM(               /* pose component for map initialization */ \
  map_init_pose_pitch,  \
  0., double) \
  CXT_ELEM(               /* pose component for map initialization */ \
  map_init_pose_yaw, \
  -TF2SIMD_HALF_PI, double) \
  CXT_ELEM(               /* length of a side of a marker in meters */ \
  marker_length,  \
  0.1627, double) \
  /* End of list */

#define CXT_MACRO_ALL_MEMBERS \
  CXT_MEMBER(             /* A transform derived from individual parameters */ \
  map_init_transform,  \
  TransformWithCovariance) \
  /* End of list */

  struct VmapContext
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

#endif //FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
