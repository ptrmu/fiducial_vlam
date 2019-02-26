
#include "map.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// Observations class
// ==============================================================================

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

// ==============================================================================
// Map class
// ==============================================================================

  Map::Map()
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

  Map::Map(const fiducial_vlam_msgs::msg::Map &msg)
  {
    marker_length_ = msg.marker_length;
    for (int i = 0; i < msg.ids.size(); i += 1) {
      Marker marker(msg.ids[i], to_TransformWithCovariance(msg.poses[i]));
      marker.set_is_fixed(msg.fixed_flags[i] != 0);
      markers_[marker.id()] = marker;
    }
  }

  std::unique_ptr<fiducial_vlam_msgs::msg::Map>
  Map::to_map_msg(const std_msgs::msg::Header &header_msg, double marker_length)
  {
    auto map_msg_unique = std::make_unique<fiducial_vlam_msgs::msg::Map>();
    auto &map_msg = *map_msg_unique;
    for (auto &marker_pair : markers_) {
      auto &marker = marker_pair.second;
      map_msg.ids.emplace_back(marker.id());
      map_msg.poses.emplace_back(to_PoseWithCovariance_msg(marker.t_map_marker()));
      map_msg.fixed_flags.emplace_back(marker.is_fixed() ? 1 : 0);
    }
    map_msg.header = header_msg;
    map_msg.marker_length = marker_length;
    return map_msg_unique;
  }

  Marker *Map::find_marker(int id)
  {
    auto marker_pair = markers_.find(id);
    return marker_pair == markers_.end() ? nullptr : &marker_pair->second;
  }

  void Map::add_marker(int id, Marker marker)
  {
    assert(markers_.count(id) == 0);
    markers_.emplace(id, std::move(marker));
  }

//  static void to_YAML_header(const Map &map, YAML::Emitter &out)
//  {
//    out << YAML::Key << "marker_length" << YAML::Value << map.marker_length();
//  }
//
//  static void to_YAML_marker(Marker &marker, YAML::Emitter &out)
//  {
//    out << YAML::BeginMap;
//    out << YAML::Key << "id" << YAML::Value << marker.id();
//    out << YAML::Key << "u" << YAML::Value << marker.update_count();
//    out << YAML::Key << "f" << YAML::Value << (marker.is_fixed() ? 1 : 0);
//    auto &t = marker.t_map_marker().transform().getOrigin();
//    out << YAML::Key << "t" << YAML::Value << YAML::Flow << YAML::BeginSeq << t.x() << t.y() << t.z() << YAML::EndSeq;
//    tf2::Quaternion q;
//    marker.t_map_marker().transform().getBasis().getRotation(q);
//    out << YAML::Key << "q" << YAML::Value << YAML::Flow << YAML::BeginSeq << q.x() << q.y() << q.z() << q.w()
//        << YAML::EndSeq;
//    out << YAML::EndMap;
//  }
//
//  static void to_YAML_markers(const std::map<int, Marker> &markers_map, YAML::Emitter &out)
//  {
//    out << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
//    for (auto marker_pair : markers_map) {
//      auto &marker = marker_pair.second;
//      to_YAML_marker(marker, out);
//    }
//    out << YAML::EndSeq;
//  }
//
//  static void to_YAML_map(const Map &map, YAML::Emitter &out)
//  {
//    out << YAML::BeginMap;
//    to_YAML_header(map, out);
//    to_YAML_markers(map.markers(), out);
//    out << YAML::EndMap;
//  }
//

//  static bool from_YAML_marker(Map &map, YAML::Node &marker_node)
//  {
//    auto id = marker_node["id"].as<int>();
//    auto update_count = marker_node["u"].as<int>();
//    auto is_fixed = marker_node["f"].as<int>();
//    auto t_node = marker_node["t"].as<YAML::Node>();
//    tf2::tf2Vector4 t(t_node[0].as<double>(), t_node[1].as<double>(), t_node[2].as<double>(), 0.0);
//    tf2::Quaternion q;
//    auto q_node = marker_node["q"].as<YAML::Node>();
//    q.setX(q_node[0].as<double>());
//    q.setY(q_node[1].as<double>());
//    q.setZ(q_node[2].as<double>());
//    q.setW(q_node[3].as<double>());
//
//    Marker marker(id, TransformWithCovariance(tf2::Transform(q, t)));
//    marker.set_is_fixed(is_fixed);
//    marker.set_update_count(update_count);
//    //markers_[id] = marker;
//    return false;
//  }
//
//  static bool from_YAML_markers(Map &map, YAML::Node &markers_node)
//  {
//    for (YAML::const_iterator it = markers_node.begin(); it != markers_node.end(); ++it) {
//      YAML::Node marker_node = *it;
//      if (marker_node.IsMap()) {
//        return from_YAML_marker(map, marker_node);
//      }
//    }
//    return false;
//  }
//
//  static bool from_YAML_map(Map &map, YAML::Node &map_node)
//  {
//    if (map_node.IsMap()) {
//      auto marker_length_node = map_node["marker_length"];
//      if (marker_length_node.IsScalar()) {
//        map.set_marker_length(marker_length_node.as<double>());
//        auto markers_node = map_node["markers"];
//        if (markers_node.IsSequence()) {
//          return from_YAML_markers(map, markers_node);
//        }
//      }
//    }
//    return false;
//  }

// ==============================================================================
// Localizer class
// ==============================================================================

  Localizer::Localizer(const std::shared_ptr<Map> &map)
    : map_(map)
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
      auto marker_ptr = map_->find_marker(observation.id());
      if (marker_ptr) {
        auto &marker = *marker_ptr;

        // Find the camera marker transform
        auto t_camera_marker = fm.solve_t_camera_marker(observation, map_->marker_length());

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

    return average_t_map_camera;
  }

// ==============================================================================
// Utility
// ==============================================================================

//  void log_tf_transform(rclcpp::Node &node, std::string s, const tf2::Transform &transform)
//  {
//    auto t = transform.getOrigin();
//    double r, p, y;
//    transform.getBasis().getRPY(r, p, y);
//
//    RCLCPP_DEBUG(node.get_logger(), "%s xyz:%lf %lf %lf, rpy:%lf %lf %lf",
//                 s.c_str(), t.x(), t.y(), t.z(), r, p, y);
//  }


}

