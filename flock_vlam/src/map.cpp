
#include "map.hpp"

#include "opencv2/calib3d.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_util.hpp"

#include "yaml-cpp/yaml.h"


namespace flock_vlam
{

//=============
// TransformWithCovariance class
//=============

  TransformWithCovariance::TransformWithCovariance(geometry_msgs::msg::PoseWithCovariance pose)
    : is_valid_(true), transform_(), variance_(0)
  {
    fromMsg(pose.pose, transform_);
  }

  geometry_msgs::msg::Pose TransformWithCovariance::to_pose_msg()
  {
    geometry_msgs::msg::Pose pose;
    toMsg(transform_, pose);
    return pose;
  }

  geometry_msgs::msg::PoseWithCovariance TransformWithCovariance::to_pose_with_covariance_msg()
  {
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.pose = to_pose_msg();
    //msg.set__covariance() // ToDo move over the covariance
    return msg;
  }

  geometry_msgs::msg::PoseStamped TransformWithCovariance::to_pose_stamped_msg(std_msgs::msg::Header &header)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header = header;
    msg.pose = to_pose_msg();
    return msg;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped TransformWithCovariance::to_pose_with_covariance_stamped_msg(
    std_msgs::msg::Header &header)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header = header;
    msg.pose = to_pose_with_covariance_msg();
    return msg;
  }

  void TransformWithCovariance::update_simple_average(TransformWithCovariance &newVal, int previous_update_count)
  {
    double previous_weight = double(previous_update_count) / (previous_update_count + 1);
    double current_weight = 1.0 / (previous_update_count + 1);

    transform_.setOrigin(transform_.getOrigin() * previous_weight +
                         newVal.transform_.getOrigin() * current_weight);

    tf2::Quaternion q1 = transform_.getRotation();
    tf2::Quaternion q2 = newVal.transform_.getRotation();
    transform_.setRotation(q1.slerp(q2, current_weight).normalized());
  }

//=============
// Observations class
//=============

  Observations::Observations(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners)
  {
    for (int i = 0; i < ids.size(); i += 1) {
      observations_.push_back(Observation(ids[i], corners[i]));
    }
  }

  Observations::Observations(const flock_vlam_msgs::msg::Observations &msg)
  {
    for (auto o : msg.observations) {

      observations_.push_back(Observation(o));
    }
  }

  flock_vlam_msgs::msg::Observations Observations::to_msg(const std_msgs::msg::Header &header_msg,
                                                          const sensor_msgs::msg::CameraInfo &camera_info_msg)
  {
    flock_vlam_msgs::msg::Observations msg;
    msg.header = header_msg;
    msg.camera_info = camera_info_msg;
    for (auto observation : observations_) {
      flock_vlam_msgs::msg::Observation obs_msg;
      obs_msg.id = observation.id();
      obs_msg.x0 = observation.corners_f_image()[0].x;
      obs_msg.x1 = observation.corners_f_image()[1].x;
      obs_msg.x2 = observation.corners_f_image()[2].x;
      obs_msg.x3 = observation.corners_f_image()[3].x;
      obs_msg.y0 = observation.corners_f_image()[0].y;
      obs_msg.y1 = observation.corners_f_image()[1].y;
      obs_msg.y2 = observation.corners_f_image()[2].y;
      obs_msg.y3 = observation.corners_f_image()[3].y;
      msg.observations.push_back(obs_msg);
    }
    return msg;
  }

//=============
// Marker class
//=============

  std::vector<cv::Point3d> Marker::corners_f_map(float marker_length)
  {
    // Build up a list of the corner locations in the map frame.
    tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
    tf2::Vector3 corner1_f_marker(marker_length / 2.f, marker_length / 2.f, 0.f);
    tf2::Vector3 corner2_f_marker(marker_length / 2.f, -marker_length / 2.f, 0.f);
    tf2::Vector3 corner3_f_marker(-marker_length / 2.f, -marker_length / 2.f, 0.f);

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

  std::vector<cv::Point3d> Marker::corners_f_marker(float marker_length)
  {
    // Build up a list of the corner locations in the map frame.
    std::vector<cv::Point3d> corners_f_marker;
    corners_f_marker.push_back(cv::Point3d(-marker_length / 2.f, marker_length / 2.f, 0.f));
    corners_f_marker.push_back(cv::Point3d(marker_length / 2.f, marker_length / 2.f, 0.f));
    corners_f_marker.push_back(cv::Point3d(marker_length / 2.f, -marker_length / 2.f, 0.f));
    corners_f_marker.push_back(cv::Point3d(-marker_length / 2.f, -marker_length / 2.f, 0.f));
    return corners_f_marker;
  }

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
    tf2::Transform first_marker_transform(q, t);
    auto first_marker_transform_with_covariance = TransformWithCovariance(first_marker_transform, 0.0);
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

  void Map::load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg)
  {
    marker_length_ = msg->marker_length;
    markers_.clear();
    for (int i = 0; i < msg->ids.size(); i += 1) {
      Marker marker(msg->ids[i], TransformWithCovariance(msg->poses[i]));
      marker.set_is_fixed(msg->fixed_flags[i] != 0);
      markers_[marker.id()] = marker;
    }
  }

  flock_vlam_msgs::msg::Map Map::to_map_msg(const std_msgs::msg::Header &header_msg, float marker_length)
  {
    flock_vlam_msgs::msg::Map map_msg;
    for (auto marker_pair : markers_) {
      auto &marker = marker_pair.second;
      map_msg.ids.push_back(marker.id());
      map_msg.poses.push_back(marker.marker_pose_f_map().to_pose_with_covariance_msg());
      map_msg.fixed_flags.push_back(marker.is_fixed() ? 1 : 0);
    }
    map_msg.header = header_msg;
    map_msg.marker_length = marker_length;
    return map_msg;
  }

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
            Marker marker(id, TransformWithCovariance(tf2::Transform(q, t), 0.0));
            marker.set_is_fixed(is_fixed);
            marker.set_update_count(update_count);
            markers_[id] = marker;
          }
        }
      }
    }
  }

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

  TransformWithCovariance Localizer::average_camera_pose_f_map(Observations &observations,
                                                               const cv::Mat &camera_matrix,
                                                               const cv::Mat &dist_coeffs)
  {
    TransformWithCovariance average_t_map_camera;
    int observations_count{0};

    // For each observation.
    for (auto observation : observations.observations()) {

      // Find the marker with the same id
      auto marker_pair = map_.markers().find(observation.id());
      if (marker_pair != map_.markers().end()) {
        auto &marker = marker_pair->second;

        // Build up two lists of corner points: 2D in the image frame, 3D in the map frame
        std::vector<cv::Point3d> all_corners_f_map = marker.corners_f_map(map_.marker_length());
        std::vector<cv::Point2f> all_corners_f_image = observation.corners_f_image();

        // Figure out image location.
        cv::Vec3d rvec, tvec;
        cv::solvePnP(all_corners_f_map, all_corners_f_image, camera_matrix, dist_coeffs, rvec, tvec);

        // Take the inverse of the returned t_camera_map to get t_map_camera;
        TransformWithCovariance t_map_camera(tf2_util::to_tf2_transform(rvec, tvec).inverse(), 0.);

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

    return average_t_map_camera;
  }

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
    auto t_map_camera = tf2_util::to_tf2_transform(rvec, tvec).inverse();

    // ToDo: get some covariance estimate
    return TransformWithCovariance(t_map_camera, 0.0);
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
        tf2_util::to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

        // Save this transform
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
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

//=============
// Utility
//=============

  void log_tf_transform(rclcpp::Node &node, const std::string s, const tf2::Transform &transform)
  {
    auto t = transform.getOrigin();
    double r, p, y;
    transform.getBasis().getRPY(r, p, y);

    RCLCPP_INFO(node.get_logger(), "%s xyz:%lf %lf %lf, rpy:%lf %lf %lf",
                s.c_str(), t.x(), t.y(), t.z(), r, p, y);
  }


}

