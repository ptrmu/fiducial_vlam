
#include "map.hpp"

#include "opencv2/calib3d.hpp"

namespace flock_vlam
{

  geometry_msgs::msg::PoseWithCovarianceStamped TransformWithCovariance::to_msg(
    std_msgs::msg::Header & header)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.set__header(header);
    msg.set__pose(to_msg());
    return msg;
  }

  geometry_msgs::msg::PoseWithCovariance TransformWithCovariance::to_msg()
  {
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.set__pose(eigen_util::to_pose(transform_));
    //msg.set__covariance()
    return msg;
  }

  Observations::Observations(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners)
  {

  }

  flock_vlam_msgs::msg::Observations Observations::to_msg(geometry_msgs::msg::PoseWithCovarianceStamped & t_map_camera)
  {
    flock_vlam_msgs::msg::Observations msg;
    return msg;
  }

  Map::Map(rclcpp::Node & node)
  : node_(node), markers_()
  {
  }

  void Map::load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg)
  {
  }

  TransformWithCovariance Map::estimate_t_map_camera(Observations & observations, float marker_length,
                                                     cv::Mat camera_matrix, cv::Mat dist_coeffs)
  {
    // Find the ids of markers that we can see and that we have a location for.
    std::set<int> good_markers;
    auto o_list = observations.observations();
    for (auto it = o_list.begin(); it != o_list.end(); ++it) {
      auto marker = markers_.find(it->id());
      if (marker != markers_.end()) {
        good_markers.insert(it->id());
      }
    }

    // If no markers are found for which we know the location return an invalid TransformWithCovariance.
    if (good_markers.size() < 1) {
      return TransformWithCovariance();
    }

    // Build up two lists of corner points: 2D in the image frame, 3D in the map frame
    std::vector<cv::Point3d> all_corners_map_corner;
    std::vector<cv::Point2f> all_corners_image_corner;
    for (auto it = o_list.begin(); it != o_list.end(); ++it)
    {
      if (good_markers.count(it->id()) > 0) {
        auto corners_map_corner = markers_.find(it->id())->second.corners_map_corner(marker_length);
        auto corners_image_corner = it->corners_image_corner();

        all_corners_map_corner.insert(all_corners_map_corner.end(),
                                      corners_map_corner.begin(), corners_map_corner.end());
        all_corners_image_corner.insert(all_corners_image_corner.end(),
                                        corners_image_corner.begin(), corners_image_corner.end());
      }
    }

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::solvePnP(all_corners_map_corner, all_corners_image_corner, camera_matrix, dist_coeffs, rvec, tvec);

    return TransformWithCovariance(rvec, tvec, 0.0);
  }
}
