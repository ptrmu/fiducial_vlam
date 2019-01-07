
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
      corners_f_map.push_back(eigen_util::to_cv_Point3d(tf_map_marker * corner_f_marker));
    }
    return corners_f_map;
  }

  Map::Map(rclcpp::Node & node)
  : node_(node), markers_()
  {

    // Create one entry in the map for now while debugging.
    auto first_marker_id = 4;
    Eigen::Vector3d t { 0, 0, 1 };
//    t.x() = 0;
//    t.y() = 0;
//    t.z() = 1;
    t.x() = 10;
    t.y() = 0;
    t.z() = 0;
    Eigen::Quaterniond q;
//    q.x() = 0.5;
//    q.y() = -0.5;
//    q.z() = -0.5;
//    q.w() = 0.5;
    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1;
    auto first_marker_transform = Eigen::Affine3d();
    first_marker_transform.translation() = t;
    first_marker_transform.linear() = q.toRotationMatrix();
    auto first_marker_transform_with_covariance = TransformWithCovariance(first_marker_transform, 0.0);
    Marker first_marker(first_marker_id, first_marker_transform_with_covariance);
    markers_[first_marker_id] = first_marker;
  }

  void Map::load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg)
  {
  }

  TransformWithCovariance Map::estimate_camera_pose_f_map(Observations &observations, float marker_length,
                                                          const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
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

    // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
    // camera coordinate system". In our case the map frame is the model coordinate system.
    // So rvec, tvec are the transformation t_camera_map. This function returns camera_pose_f_map
    // or equivalently t_map_camera. Invert the rvec, tvec transform before returning it.
    auto t_map_camera = eigen_util::to_affine(rvec, tvec).inverse(Eigen::TransformTraits::Isometry);


//    auto &c = all_corners_f_image;
//    RCLCPP_INFO(node_.get_logger(), "corners 0:%f,%f 1:%f,%f 2:%f,%f 3:%f,%f",
//               c[0].x, c[0].y, c[1].x, c[1].y, c[2].x, c[2].y, c[3].x, c[3].y);
//    log_transform("t_map_camera", t_map_camera);

    // ToDo: get some covariance estimate
    return TransformWithCovariance(t_map_camera, 0.0);
  }

  // Compute marker poses using Map info. Note this can only be done if
  // a camera pose in the map frame is determined and we have a marker's pose in
  // the map frame. The calculation is to take the marker location in the map
  // frame t_map_marker and transform (pre-multiply) it by t_map_camera.inverse()
  // to get t_camera_marker.
  void Map::markers_pose_f_camera(const TransformWithCovariance &camera_pose_f_map, const std::vector<int> &ids,
                             std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
  {
    // Can not do this calculation without a camera pose.
    if (!camera_pose_f_map.is_valid()) {
      return;
    }

    auto t_camera_map = camera_pose_f_map.transform().inverse(Eigen::TransformTraits::Isometry);

    // Loop through the ids of the markers visible in this image
    for (auto id : ids) {
      auto marker = markers_.find(id);
      if (marker != markers_.end()) {

        // Found a marker that is visible in the image and we have a pose_f_map.
        // Calculate marker_pose_f_camera or equivalently t_camera_marker
        auto t_map_marker = marker->second.marker_pose_f_map().transform();
        auto t_camera_marker = t_camera_map * t_map_marker;

        // Convert the pose to an OpenCV transform
        cv::Vec3d rvec, tvec;
        eigen_util::to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

        // Save this transform
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
      }
    }
  }

  void Map::log_transform(std::string prefix, Eigen::Affine3d transform)
  {
    auto rpy = transform.linear().eulerAngles(0, 1, 2);
    auto t = transform.translation();
    RCLCPP_INFO(node_.get_logger(), "%s x,y,z:%lf,%lf,%lf r,p,y:%lf,%lf,%lf", prefix.c_str(),
      t.x(), t.y(), t.z(), rpy[0], rpy[1], rpy[2]);
  }
}

