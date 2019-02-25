
#include "map.hpp"

#include "convert_util.hpp"

// todo remove this
//#include "opencv2/calib3d.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"


namespace fiducial_vlam
{

//=============
// TransformWithCovariance class
//=============
//
//  TransformWithCovariance::TransformWithCovariance(geometry_msgs::msg::PoseWithCovariance pose)
//    : is_valid_(true), transform_(), variance_(0)
//  {
//    fromMsg(pose.pose, transform_);
//  }
//
//  geometry_msgs::msg::Pose TransformWithCovariance::to_Pose_msg()
//  {
//    geometry_msgs::msg::Pose pose;
//    toMsg(transform_, pose);
//    return pose;
//  }
//
//  geometry_msgs::msg::PoseWithCovariance TransformWithCovariance::to_pose_with_covariance_msg()
//  {
//    geometry_msgs::msg::PoseWithCovariance msg;
//    msg.pose = to_Pose_msg();
//    //msg.set__covariance() // ToDo move over the covariance
//    return msg;
//  }
//
//  geometry_msgs::msg::PoseStamped TransformWithCovariance::to_pose_stamped_msg(std_msgs::msg::Header &header)
//  {
//    geometry_msgs::msg::PoseStamped msg;
//    msg.header = header;
//    msg.pose = to_Pose_msg();
//    return msg;
//  }
//
//  geometry_msgs::msg::PoseWithCovarianceStamped TransformWithCovariance::to_pose_with_covariance_stamped_msg(
//    std_msgs::msg::Header &header)
//  {
//    geometry_msgs::msg::PoseWithCovarianceStamped msg;
//    msg.header = header;
//    msg.pose = to_pose_with_covariance_msg();
//    return msg;
//  }
//
//  void TransformWithCovariance::update_simple_average(TransformWithCovariance &newVal, int previous_update_count)
//  {
//    double previous_weight = double(previous_update_count) / (previous_update_count + 1);
//    double current_weight = 1.0 / (previous_update_count + 1);
//
//    transform_.setOrigin(transform_.getOrigin() * previous_weight +
//                         newVal.transform_.getOrigin() * current_weight);
//
//    tf2::Quaternion q1 = transform_.getRotation();
//    tf2::Quaternion q2 = newVal.transform_.getRotation();
//    transform_.setRotation(q1.slerp(q2, current_weight).normalized());
//  }

//=============
// Observations class
//=============

// todo remove this
//  Observations::Observations(const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners)
//  {
//    assert(ids.size() == corners.size());
//    for (int i = 0; i < ids.size(); i += 1) {
//      observations_.emplace_back(Observation(ids[i], corners[i]));
//    }
//  }

  Observations::Observations(const fiducial_vlam_msgs::msg::Observations &msg)
  {
    for (auto &obs : msg.observations) {
      observations_.emplace_back(Observation(obs.id,
                                             obs.x0, obs.y0,
                                             obs.x1, obs.y1,
                                             obs.x2, obs.y2,
                                             obs.x3, obs.y3));
    }
  }

  fiducial_vlam_msgs::msg::Observations Observations::to_msg(const std_msgs::msg::Header &header_msg,
                                                             const sensor_msgs::msg::CameraInfo &camera_info_msg)
  {
    fiducial_vlam_msgs::msg::Observations msg;
    msg.header = header_msg;
    msg.camera_info = camera_info_msg;
    for (auto observation : observations_) {
      fiducial_vlam_msgs::msg::Observation obs_msg;
      obs_msg.id = observation.id();
      obs_msg.x0 = observation.x0();
      obs_msg.x1 = observation.x1();
      obs_msg.x2 = observation.x2();
      obs_msg.x3 = observation.x3();
      obs_msg.y0 = observation.y0();
      obs_msg.y1 = observation.y1();
      obs_msg.y2 = observation.y2();
      obs_msg.y3 = observation.y3();
      msg.observations.emplace_back(obs_msg);
    }
    return msg;
  }

//=============
// Marker class
//=============

// todo remove this
//  std::vector<cv::Point3d> Marker::corners_f_map(float marker_length)
//  {
//    // Build up a list of the corner locations in the map frame.
//    tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
//    tf2::Vector3 corner1_f_marker(marker_length / 2.f, marker_length / 2.f, 0.f);
//    tf2::Vector3 corner2_f_marker(marker_length / 2.f, -marker_length / 2.f, 0.f);
//    tf2::Vector3 corner3_f_marker(-marker_length / 2.f, -marker_length / 2.f, 0.f);
//
//    auto t_map_marker_tf = t_map_marker().transform();
//    auto corner0_f_map = t_map_marker_tf * corner0_f_marker;
//    auto corner1_f_map = t_map_marker_tf * corner1_f_marker;
//    auto corner2_f_map = t_map_marker_tf * corner2_f_marker;
//    auto corner3_f_map = t_map_marker_tf * corner3_f_marker;
//
//    std::vector<cv::Point3d> corners_f_map;
//    corners_f_map.emplace_back(cv::Point3d(corner0_f_map.x(), corner0_f_map.y(), corner0_f_map.z()));
//    corners_f_map.emplace_back(cv::Point3d(corner1_f_map.x(), corner1_f_map.y(), corner1_f_map.z()));
//    corners_f_map.emplace_back(cv::Point3d(corner2_f_map.x(), corner2_f_map.y(), corner2_f_map.z()));
//    corners_f_map.emplace_back(cv::Point3d(corner3_f_map.x(), corner3_f_map.y(), corner3_f_map.z()));
//
//    return corners_f_map;
//  }
//
//  std::vector<cv::Point3d> Marker::corners_f_marker(float marker_length)
//  {
//    // Build up a list of the corner locations in the map frame.
//    std::vector<cv::Point3d> corners_f_marker;
//    corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, marker_length / 2.f, 0.f));
//    corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, marker_length / 2.f, 0.f));
//    corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, -marker_length / 2.f, 0.f));
//    corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, -marker_length / 2.f, 0.f));
//    return corners_f_marker;
//  }

//=============
// Map class
//=============

  Map::Map(rclcpp::Node &node)
    : node_(node)
  {

    // Create one entry in the map for now while debugging.
    auto first_marker_id = 1;
    tf2::Vector3 t{0, 0, 1};
    tf2::Quaternion q;
    q.setX(0.5);
    q.setY(-0.5);
    q.setZ(-0.5);
    q.setW(0.5);
//     tf2::Vector3 t{0, 0, 0};
//     tf2::Quaternion q;
//     q.setX(0);
//     q.setY(0);
//     q.setZ(0);
//     q.setW(1);
    tf2::Transform first_marker_transform(q, t);
    auto first_marker_transform_with_covariance = TransformWithCovariance(first_marker_transform);
    Marker first_marker(first_marker_id, first_marker_transform_with_covariance);
    first_marker.set_is_fixed(true);
    markers_[first_marker_id] = first_marker;

    marker_length_ = 0.162718;

//    std::string yaml;
//    to_YAML_string(yaml);
//    RCLCPP_INFO(node_.get_logger(), "yaml %s", yaml.c_str());
//
//    from_YAML_string(yaml);
  }

  void Map::load_from_msg(const fiducial_vlam_msgs::msg::Map &msg)
  {
    marker_length_ = msg.marker_length;
    markers_.clear();
    for (int i = 0; i < msg.ids.size(); i += 1) {
      Marker marker(msg.ids[i], to_TransformWithCovariance(msg.poses[i]));
      marker.set_is_fixed(msg.fixed_flags[i] != 0);
      markers_[marker.id()] = marker;
    }
  }

  fiducial_vlam_msgs::msg::Map Map::to_map_msg(const std_msgs::msg::Header &header_msg, double marker_length)
  {
    fiducial_vlam_msgs::msg::Map map_msg;
    for (auto &marker_pair : markers_) {
      auto &marker = marker_pair.second;
      map_msg.ids.emplace_back(marker.id());
      map_msg.poses.emplace_back(to_PoseWithCovariance_msg(marker.marker_pose_f_map()));
      map_msg.fixed_flags.emplace_back(marker.is_fixed() ? 1 : 0);
    }
    map_msg.header = header_msg;
    map_msg.marker_length = marker_length;
    return map_msg;
  }

#if 0
  void Map::load_from_file(std::string full_path)
  {
    YAML::Node config = YAML::LoadFile("config.yaml");
// catch YAML::BadFile
  }

  void Map::to_YAML_string(std::string &yaml)
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "marker_length" << YAML::Value << marker_length_;
    out << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
    for (auto marker_pair : markers_) {
      auto &marker = marker_pair.second;
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << marker.id();
      out << YAML::Key << "u" << YAML::Value << marker.update_count();
      out << YAML::Key << "f" << YAML::Value << (marker.is_fixed() ? 1 : 0);
      auto &t = marker.marker_pose_f_map().transform().getOrigin();
      out << YAML::Key << "t" << YAML::Value << YAML::Flow << YAML::BeginSeq << t.x() << t.y() << t.z() << YAML::EndSeq;
      tf2::Quaternion q;
      marker.marker_pose_f_map().transform().getBasis().getRotation(q);
      out << YAML::Key << "q" << YAML::Value << YAML::Flow << YAML::BeginSeq << q.x() << q.y() << q.z() << q.w()
          << YAML::EndSeq;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    yaml.append(out.c_str());
  }

  void Map::from_YAML_string(std::string &yaml)
  {
    YAML::Node node = YAML::Load(yaml);
    if (node.IsMap()) {
      marker_length_ = node["marker_length"].as<double>();
      YAML::Node markers_node = node["markers"].as<YAML::Node>();
      if (markers_node.IsSequence()) {
        markers_.clear(); // clear out the member map of markers.
        for (YAML::const_iterator it = markers_node.begin(); it != markers_node.end(); ++it) {
          YAML::Node marker_node = *it;
          if (marker_node.IsMap()) {
            auto id = marker_node["id"].as<int>();
            auto update_count = marker_node["u"].as<int>();
            auto is_fixed = marker_node["f"].as<int>();
            auto t_node = marker_node["t"].as<YAML::Node>();
            tf2::tf2Vector4 t(t_node[0].as<double>(), t_node[1].as<double>(), t_node[2].as<double>(), 0.0);
            tf2::Quaternion q;
            auto q_node = marker_node["q"].as<YAML::Node>();
            q.setX(q_node[0].as<double>());
            q.setY(q_node[1].as<double>());
            q.setZ(q_node[2].as<double>());
            q.setW(q_node[3].as<double>());
            Marker marker(id, TransformWithCovariance(tf2::Transform(q, t)));
            marker.set_is_fixed(is_fixed);
            marker.set_update_count(update_count);
            markers_[id] = marker;
          }
        }
      }
    }
  }
#endif

  void Map::save_to_file(std::string full_path)
  {
  }

//=============
// Localizer class
//=============

  Localizer::Localizer(rclcpp::Node &node, Map &map)
    : node_(node), map_(map)
  {
  }

  TransformWithCovariance Localizer::average_t_map_camera(Observations &observations,
                                                          FiducialMath &fm)
  {
    TransformWithCovariance average_t_map_camera{};
    int observations_count{0};

    // For each observation.
    for (auto observation : observations.observations()) {

      // Find the marker with the same id
      auto marker_pair = map_.markers().find(observation.id());
      if (marker_pair != map_.markers().end()) {
        auto &marker = marker_pair->second;

        // Find the camera marker transform
        auto t_camera_marker = fm.solve_t_camera_marker(observation, map_.marker_length());

        // The solvePnP function returns the marker in the camera frame: t_camera_marker
        auto tf2_t_marker_camera = t_camera_marker.transform().inverse();
        auto tf2_t_map_camera = marker.t_map_marker().transform() * tf2_t_marker_camera;
        TransformWithCovariance t_map_camera(tf2_t_map_camera);

        // Average this new measurement with the previous
        if (observations_count == 0) {
          average_t_map_camera = t_map_camera;
          observations_count += 1;
        } else {
          average_t_map_camera.update_simple_average(t_map_camera, observations_count);
          observations_count += 1;
        }
      }
    }

    RCLCPP_DEBUG(node_.get_logger(), "Camera pose. Averaged from %d observations", observations_count);
    log_tf_transform(node_, "", average_t_map_camera.transform());

    return average_t_map_camera;
  }

#if 0
// This routine does not seem to work. The average version is being used. Maybe someday
// this can be debugged.
  TransformWithCovariance Localizer::estimate_camera_pose_f_map(Observations &observations,
                                                                const cv::Mat &camera_matrix,
                                                                const cv::Mat &dist_coeffs)
  {
    // Find the ids of markers that we can see and that we have a location for.
    std::set<int> good_markers;
    for (auto o : observations.observations()) {
      auto marker = map_.markers().find(o.id());
      if (marker != map_.markers().end()) {
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
    for (auto o : observations.observations()) {
      if (good_markers.count(o.id()) > 0) {
        auto corners_f_map = map_.markers().find(o.id())->second.corners_f_map(map_.marker_length());
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
    auto t_map_camera = to_tf2_transform(rvec, tvec).inverse();

    // ToDo: get some covariance estimate
    return TransformWithCovariance(t_map_camera);
  }

// Compute marker poses using Map info. Note this can only be done if
// a camera pose in the map frame is determined and we have a marker's pose in
// the map frame. The calculation is to take the marker location in the map
// frame t_map_marker and transform (pre-multiply) it by t_map_camera.inverse()
// to get t_camera_marker.
  void Localizer::markers_pose_f_camera(const TransformWithCovariance &camera_pose_f_map, const std::vector<int> &ids,
                                        std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
  {
    // Can not do this calculation without a camera pose.
    if (!camera_pose_f_map.is_valid()) {
      return;
    }

    auto t_camera_map = camera_pose_f_map.transform().inverse();

    // Loop through the ids of the markers visible in this image
    for (auto id : ids) {
      auto marker = map_.markers().find(id);
      if (marker != map_.markers().end()) {

        // Found a marker that is visible in the image and we have a pose_f_map.
        // Calculate marker_pose_f_camera or equivalently t_camera_marker
        auto t_map_marker = marker->second.marker_pose_f_map().transform();
        auto t_camera_marker = t_camera_map * t_map_marker;

        // Convert the pose to an OpenCV transform
        cv::Vec3d rvec, tvec;
        to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

        // Save this transform
        rvecs.emplace_back(rvec);
        tvecs.emplace_back(tvec);
      }
    }
  }

// This method was used to debug transforms. It is not used but will be kept around for awhile.
  void Localizer::markers_pose_f_camera_tf2(Observations &observations,
                                            const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                                            std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
  {
    // Loop through the ids of the markers visible in this image
    for (auto observation : observations.observations()) {
      auto marker = map_.markers().find(observation.id());
      if (marker != map_.markers().end()) {

        // Found a marker that is visible in the image and we have a pose_f_map or t_map_marker.
        auto t_map_marker_tf = marker->second.marker_pose_f_map().transform();

        // Build up a list of the corner locations in the map frame.
        tf2::Vector3 corner0_f_marker(-map_.marker_length() / 2.f, map_.marker_length() / 2.f, 0.f);
        tf2::Vector3 corner1_f_marker(map_.marker_length() / 2.f, map_.marker_length() / 2.f, 0.f);
        tf2::Vector3 corner2_f_marker(map_.marker_length() / 2.f, -map_.marker_length() / 2.f, 0.f);
        tf2::Vector3 corner3_f_marker(-map_.marker_length() / 2.f, -map_.marker_length() / 2.f, 0.f);

        auto corner0_f_map = t_map_marker_tf * corner0_f_marker;
        auto corner1_f_map = t_map_marker_tf * corner1_f_marker;
        auto corner2_f_map = t_map_marker_tf * corner2_f_marker;
        auto corner3_f_map = t_map_marker_tf * corner3_f_marker;

        std::vector<cv::Point3d> world_points;
        world_points.emplace_back(cv::Point3d(corner0_f_map.x(), corner0_f_map.y(), corner0_f_map.z()));
        world_points.emplace_back(cv::Point3d(corner1_f_map.x(), corner1_f_map.y(), corner1_f_map.z()));
        world_points.emplace_back(cv::Point3d(corner2_f_map.x(), corner2_f_map.y(), corner2_f_map.z()));
        world_points.emplace_back(cv::Point3d(corner3_f_map.x(), corner3_f_map.y(), corner3_f_map.z()));

        // get a list of the 2D corner locations in the image frame
        auto imagePoints = observation.corners_f_image();

        // Figure out the map to camera transform
        cv::Vec3d rvec, tvec;
        cv::solvePnP(world_points, imagePoints, camera_matrix, dist_coeffs, rvec, tvec);

        tf2::Transform t_camera_map_tf(to_tf2_transform(rvec, tvec));

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
        rvecs.emplace_back(rvec);
        tvecs.emplace_back(tvec);
      }
    }
  }
#endif

//=============
// Utility
//=============

  void log_tf_transform(rclcpp::Node &node, const std::string s, const tf2::Transform &transform)
  {
    auto t = transform.getOrigin();
    double r, p, y;
    transform.getBasis().getRPY(r, p, y);

    RCLCPP_DEBUG(node.get_logger(), "%s xyz:%lf %lf %lf, rpy:%lf %lf %lf",
                 s.c_str(), t.x(), t.y(), t.z(), r, p, y);
  }


}

