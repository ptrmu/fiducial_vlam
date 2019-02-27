#ifndef FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
#define FIDUCIAL_VLAM_VMAP_CONTEXT_HPP

#include "rclcpp/rclcpp.hpp"

#include "context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(marker_map_full_filename, /* name of the file to store the marker map in  */ \
  "", std::string) \
  CXT_ELEM(make_not_use_map, /* non-zero => create a new map  */ \
  1, int) \
  CXT_ELEM(map_init_style, /* 0->marker id, pose from file, 1->marker id, pose as parameter, 2->camera pose as parameter  */ \
  1, int) \
  CXT_ELEM(map_init_id, /* marker id for map initialization */ \
  1, int) \
  CXT_ELEM(map_init_pose_x, /* pose component for map initialization */ \
  0., double) \
  CXT_ELEM(map_init_pose_y, /* pose component for map initialization */ \
  0., double) \
  CXT_ELEM(map_init_pose_z, /* pose component for map initialization */ \
  1., double) \
  CXT_ELEM(map_init_pose_roll, /* pose component for map initialization */ \
  TF2SIMD_HALF_PI, double) \
  CXT_ELEM(map_init_pose_pitch, /* pose component for map initialization */ \
  0., double) \
  CXT_ELEM(map_init_pose_yaw, /* pose component for map initialization */ \
  -TF2SIMD_HALF_PI, double) \
  CXT_ELEM(marker_length, /* length of a side of a marker in meters */ \
  0.1627, double) \
  CXT_ELEM(marker_map_publish_frequency_hz, /* Hz => rate at which the marker map is published */ \
  0., double) \
  CXT_ELEM(publish_marker_tfs, /* non-zero => publish the tf of all the known markers  */ \
  1, int) \
  CXT_ELEM(publish_marker_visualizations, /* non-zero => publish a shape that represents a marker  */ \
  1, int) \
  /* End of list */

#define CXT_MACRO_ALL_MEMBERS \
  CXT_MEMBER(map_init_transform,  /* A transform derived from individual parameters */ \
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
