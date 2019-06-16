
#include "vloc_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{
  void VlocContext::load_parameters(rclcpp::Node &node)
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node, (*this), n, t, d)
    CXT_MACRO_INIT_PARAMETERS(VLOC_ALL_PARAMS, validate_parameters);

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node, VLOC_ALL_PARAMS, validate_parameters)
  }

  void VlocContext::validate_parameters()
  {
    t_camera_base_ = TransformWithCovariance(TransformWithCovariance::mu_type{
      t_camera_base_x_, t_camera_base_y_, t_camera_base_z_,
      t_camera_base_roll_, t_camera_base_pitch_, t_camera_base_yaw_});
  }
}

