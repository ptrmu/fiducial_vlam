
#include "vmap_context.hpp"

namespace fiducial_vlam
{
  void VmapContext::load_parameters(rclcpp::Node &node)
  {
    // Read parameters from the command line. NOTE: the get_parameter() method will
    // not modify the member element if the parameter does not exist on the command line.
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_LOAD_PARAM(n, a)
    CXT_MACRO_ALL_PARAMS

    validate_parameters();
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
