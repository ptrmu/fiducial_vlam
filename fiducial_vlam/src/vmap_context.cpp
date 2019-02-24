
#include "vmap_context.hpp"

namespace fiducial_vlam
{

  void VmapContext::load_parameters(rclcpp::Node &node)
  {
    // Read parameters from the command line. NOTE: the get_parameter() method will
    // not modify the member element if the parameter does not exist on the command line.

    node.get_parameter<int>("publish_marker_tfs", publish_marker_tfs_);
    node.get_parameter<int>("publish_marker_visualizations", publish_marker_visualizations_);

    if (std::abs(marker_map_publish_frequency_hz_) < 1.e-10) {
      marker_map_publish_frequency_hz_ = 30. / 60.;
    }
  }

}
