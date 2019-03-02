
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

  fiducial_vlam_msgs::msg::Observations Observations::to_msg(std_msgs::msg::Header::_stamp_type stamp,
                                                             const std_msgs::msg::Header::_frame_id_type &frame_id,
                                                             const sensor_msgs::msg::CameraInfo &camera_info_msg)
  {
    fiducial_vlam_msgs::msg::Observations msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.camera_info = camera_info_msg;
    for (
      auto observation: observations_) {
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
      msg.observations.
        emplace_back(obs_msg);
    }
    return
      msg;
  }

// ==============================================================================
// Map class
// ==============================================================================

  Map::Map(double marker_length)
  {
    marker_length_ = marker_length;
  }

  Map::Map(const fiducial_vlam_msgs::msg::Map &msg)
  {
    marker_length_ = msg.marker_length;
    for (int i = 0; i < msg.ids.size(); i += 1) {
      Marker marker(msg.ids[i], to_TransformWithCovariance(msg.poses[i]));
      marker.set_is_fixed(msg.fixed_flags[i] != 0);
      add_marker(std::move(marker));
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

  void Map::add_marker(Marker marker)
  {
    assert(markers_.count(marker.id()) == 0);
    markers_.emplace(marker.id(), std::move(marker));
  }

// ==============================================================================
// Localizer class
// ==============================================================================

  Localizer::Localizer(const std::unique_ptr<Map> &map)
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

