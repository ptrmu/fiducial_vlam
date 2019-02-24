#ifndef FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
#define FIDUCIAL_VLAM_VMAP_CONTEXT_HPP

#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{
  struct VmapContext
  {
    // map parameters
    std::string marker_map_full_filename_;  // name of the file to store the marker map in.

    // map creation parameters
    int make_not_use_map_ = 1; // non-zero => create a new map

    // 0->marker id, marker pose from file
    // 1->marker id, marker pose as parameter
    // 2->camera pose as parameter
    int map_init_style_;
    int map_init_id_;
    double map_init_pose_x_;
    double map_init_pose_y_;
    double map_init_pose_z_;
    double map_init_pose_r_;
    double map_init_pose_p_;
    double map_init_pose_yaw_;

    double marker_map_publish_frequency_hz_ = 30. / 60.;  // Hz => rate at which the marker map is published

    // visualization publishing parameters
    int publish_marker_tfs_ = 1;  // non-zero => publish the tf of all the known markers
    int publish_marker_visualizations_ = 1;  // non-zero => publish a shape that represents the marker

    void load_parameters(rclcpp::Node &node);
  };
}

#endif //FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
