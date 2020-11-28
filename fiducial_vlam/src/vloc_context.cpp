
#include "vloc_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{
  void VlocContext::load_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, (*this), n, t, d)
    CXT_MACRO_INIT_PARAMETERS(VLOC_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node_, (*this), VLOC_ALL_PARAMS, validate_parameters)

    RCLCPP_INFO(node_.get_logger(), "VlocNode Parameters");

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, node_.get_logger(), (*this), n, t, d)
    VLOC_ALL_PARAMS
  }

  void VlocContext::validate_parameters()
  {
    t_camera_base_ = TransformWithCovariance(TransformWithCovariance::mu_type{
      t_camera_base_x_, t_camera_base_y_, t_camera_base_z_,
      t_camera_base_roll_, t_camera_base_pitch_, t_camera_base_yaw_});
  }
}

