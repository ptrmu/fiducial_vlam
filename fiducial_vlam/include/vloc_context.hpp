#ifndef FIDUCIAL_VLAM_VLOC_CONTEXT_HPP
#define FIDUCIAL_VLAM_VLOC_CONTEXT_HPP

#include "rclcpp/rclcpp.hpp"

#include "context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(publish_camera_tf, /* non-zero => publish the tf of the camera at every frame  */ \
  1, int) \
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
