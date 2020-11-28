
#include "vmap_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{
  void VmapContext::load_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, (*this), n, t, d)
    CXT_MACRO_INIT_PARAMETERS(VMAP_ALL_PARAMS, validate_parameters)


#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node_, (*this), VMAP_ALL_PARAMS, validate_parameters)

    RCLCPP_INFO(node_.get_logger(), "VmapNode Parameters");

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, node_.get_logger(), (*this), n, t, d)
    VMAP_ALL_PARAMS
  }

  void VmapContext::validate_parameters()
  {
    if (std::abs(marker_map_publish_frequency_hz_) < 1.e-10) {
      marker_map_publish_frequency_hz_ = 30. / 60.;
    }

    map_init_transform_ = TransformWithCovariance(TransformWithCovariance::mu_type{
      map_init_pose_x_, map_init_pose_y_, map_init_pose_z_,
      map_init_pose_roll_, map_init_pose_pitch_, map_init_pose_yaw_});
  }
}
