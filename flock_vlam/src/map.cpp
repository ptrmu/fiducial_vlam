
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
    for (int i = 0; i < ids.size(); i += 1) {
      observations_.push_back(Observation(ids[i], corners[i]));
    }
  }

  flock_vlam_msgs::msg::Observations Observations::to_msg(geometry_msgs::msg::PoseWithCovarianceStamped & camera_pose_f_map_msg)
  {
    flock_vlam_msgs::msg::Observations msg;
    return msg;
  }

  std::vector<cv::Point3d> Marker::corners_f_map(float marker_length) {
    std::vector<Eigen::Vector3d> corners_f_marker {
      Eigen::Vector3d(-marker_length / 2.f, marker_length / 2.f, 0.f),
      Eigen::Vector3d( marker_length / 2.f, marker_length / 2.f, 0.f),
      Eigen::Vector3d( marker_length / 2.f,-marker_length / 2.f, 0.f),
      Eigen::Vector3d(-marker_length / 2.f,-marker_length / 2.f, 0.f),
    };
    std::vector<cv::Point3d> corners_f_map;
    auto tf_map_marker = t_map_marker().transform();
    for (auto corner_f_marker : corners_f_marker)
    {
      corners_f_map.push_back(eigen_util::to_cvPoint3d(tf_map_marker * corner_f_marker));
    }
    return corners_f_map;
  }

  Map::Map(rclcpp::Node & node)
  : node_(node), markers_()
  {
  }

  void Map::load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg)
  {
  }

  TransformWithCovariance Map::estimate_camera_pose_f_map(Observations &observations, float marker_length,
                                                          cv::Mat camera_matrix, cv::Mat dist_coeffs)
  {
    // Find the ids of markers that we can see and that we have a location for.
    std::set<int> good_markers;
    for (auto o : observations.observations()) {
      auto marker = markers_.find(o.id());
      if (marker != markers_.end()) {
        good_markers.insert(o.id());
      }
    }

    // If no markers are found for which we know the location return an invalid TransformWithCovariance.
    if (good_markers.size() < 1) {
      return TransformWithCovariance();
    }

    // Build up two lists of corner points: 2D in the image frame, 3D in the map frame
    std::vector<cv::Point3d> all_corners_f_map;
    std::vector<cv::Point2f> all_corners_f_image;
    for (auto o : observations.observations())
    {
      if (good_markers.count(o.id()) > 0) {
        auto corners_f_map = markers_.find(o.id())->second.corners_f_map(marker_length);
        auto corners_f_image = o.corners_f_image();

        all_corners_f_map.insert(all_corners_f_map.end(),
                                 corners_f_map.begin(), corners_f_map.end());
        all_corners_f_image.insert(all_corners_f_image.end(),
                                   corners_f_image.begin(), corners_f_image.end());
      }
    }

    // Figure out where the images was taken from: camera_pose_f_map.
    cv::Vec3d rvec, tvec;
    cv::solvePnP(all_corners_f_map, all_corners_f_image, camera_matrix, dist_coeffs, rvec, tvec);

    // ToDo: get some covariance estimate

    return TransformWithCovariance(rvec, tvec, 0.0);
  }
}
