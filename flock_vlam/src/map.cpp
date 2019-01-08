
#include "map.hpp"

#include "opencv2/calib3d.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_util.hpp"

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
    geometry_msgs::msg::Pose pose;
    toMsg(transform_, pose);
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.set__pose(pose);
    //msg.set__covariance() // ToDo move over the covariance
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
    // ToDo
    return msg;
  }

  std::vector<cv::Point3d> Marker::corners_f_map(float marker_length) {

    // Build up a list of the corner locations in the map frame.
    tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
    tf2::Vector3 corner1_f_marker( marker_length / 2.f, marker_length / 2.f, 0.f);
    tf2::Vector3 corner2_f_marker( marker_length / 2.f,-marker_length / 2.f, 0.f);
    tf2::Vector3 corner3_f_marker(-marker_length / 2.f,-marker_length / 2.f, 0.f);

    auto t_map_marker_tf = t_map_marker().transform();
    auto corner0_f_map = t_map_marker_tf * corner0_f_marker;
    auto corner1_f_map = t_map_marker_tf * corner1_f_marker;
    auto corner2_f_map = t_map_marker_tf * corner2_f_marker;
    auto corner3_f_map = t_map_marker_tf * corner3_f_marker;

    std::vector<cv::Point3d> corners_f_map;
    corners_f_map.push_back(cv::Point3d(corner0_f_map.x(), corner0_f_map.y(), corner0_f_map.z()));
    corners_f_map.push_back(cv::Point3d(corner1_f_map.x(), corner1_f_map.y(), corner1_f_map.z()));
    corners_f_map.push_back(cv::Point3d(corner2_f_map.x(), corner2_f_map.y(), corner2_f_map.z()));
    corners_f_map.push_back(cv::Point3d(corner3_f_map.x(), corner3_f_map.y(), corner3_f_map.z()));

    return corners_f_map;
  }

  Map::Map(rclcpp::Node & node)
  : node_(node), markers_()
  {

    // Create one entry in the map for now while debugging.
    auto first_marker_id = 4;
    tf2::Vector3 t { 0, 0, 1 };
    tf2::Quaternion q;
    q.setX( 0.5);
    q.setY(-0.5);
    q.setZ(-0.5);
    q.setW( 0.5);
    tf2::Transform first_marker_transform(q, t);
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
    auto t_map_camera = tf2_util::to_tf2_transform(rvec, tvec).inverse();

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

    auto t_camera_map = camera_pose_f_map.transform().inverse();

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
        tf2_util::to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

        // Save this transform
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
      }
    }
  }

  // This method was used to debug transforms. It is not used but will be kept around for awhile.
  void Map::markers_pose_f_camera_tf2(Observations &observations, float marker_length,
                                      const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                                      std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
  {
    // Loop through the ids of the markers visible in this image
    for (auto observation : observations.observations()) {
      auto marker = markers_.find(observation.id());
      if (marker != markers_.end()) {

        // Found a marker that is visible in the image and we have a pose_f_map or t_map_marker.
        auto t_map_marker_tf = marker->second.marker_pose_f_map().transform();

        // Build up a list of the corner locations in the map frame.
        tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
        tf2::Vector3 corner1_f_marker( marker_length / 2.f, marker_length / 2.f, 0.f);
        tf2::Vector3 corner2_f_marker( marker_length / 2.f,-marker_length / 2.f, 0.f);
        tf2::Vector3 corner3_f_marker(-marker_length / 2.f,-marker_length / 2.f, 0.f);

        auto corner0_f_map = t_map_marker_tf * corner0_f_marker;
        auto corner1_f_map = t_map_marker_tf * corner1_f_marker;
        auto corner2_f_map = t_map_marker_tf * corner2_f_marker;
        auto corner3_f_map = t_map_marker_tf * corner3_f_marker;

        std::vector<cv::Point3d> world_points;
        world_points.push_back(cv::Point3d(corner0_f_map.x(), corner0_f_map.y(), corner0_f_map.z()));
        world_points.push_back(cv::Point3d(corner1_f_map.x(), corner1_f_map.y(), corner1_f_map.z()));
        world_points.push_back(cv::Point3d(corner2_f_map.x(), corner2_f_map.y(), corner2_f_map.z()));
        world_points.push_back(cv::Point3d(corner3_f_map.x(), corner3_f_map.y(), corner3_f_map.z()));

        // get a list of the 2D corner locations in the image frame
        auto imagePoints = observation.corners_f_image();

        // Figure out the map to camera transform
        cv::Vec3d rvec, tvec;
        cv::solvePnP(world_points, imagePoints, camera_matrix, dist_coeffs, rvec, tvec);

        tf2::Transform t_camera_map_tf(tf2_util::to_tf2_transform(rvec, tvec));

        // Figure out the pose of the marker in the camera frame
        auto t_camera_marker_tf = t_camera_map_tf * t_map_marker_tf;

        // Convert to rvec, tvec
        auto t_camera_marker_tf_t = t_camera_marker_tf.getOrigin();
        tvec[0] = t_camera_marker_tf_t.x();
        tvec[1] = t_camera_marker_tf_t.y();
        tvec[2] = t_camera_marker_tf_t.z();
        auto t_camera_marker_tf_r = t_camera_marker_tf.getBasis();
        cv::Mat rmat(3, 3, CV_64FC1);
        for (int row = 0; row < 3; row++) {
          for (int col = 0; col < 3; col++) {
            rmat.at<double>(row, col) = t_camera_marker_tf_r[row][col];
          }
        }
        cv::Rodrigues(rmat, rvec);

        // Save this transform
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
      }
    }
  }
}

