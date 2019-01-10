

#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include "map.hpp"

namespace flock_vlam
{

//=============
// Mapper class
//=============

  class Mapper
  {
    rclcpp::Node &node_;
    Map &map_;

  public:
    Mapper(rclcpp::Node &node, Map &map)
      : node_(node), map_(map)
    {
    }

    virtual ~Mapper()
    {}

    auto &node() const
    { return node_; }

    auto &map()
    { return map_; }

    virtual bool
    update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations,
               const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) = 0;
  };

//=============
// MapperSimpleAverage class
//=============

  class MapperSimpleAverage : public Mapper
  {
    int observations_before_publish_base{10};
    int observations_before_publish_reset{observations_before_publish_base};
    int observations_before_publish{observations_before_publish_base};

  public:
    MapperSimpleAverage(rclcpp::Node &node, Map &map)
      : Mapper(node, map)
    {
    }

    virtual ~MapperSimpleAverage()
    {}

    virtual bool
    update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations,
               const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
    {
      bool marker_added{false};

      // For all observations estimate the marker location and update the map
      for (auto observation : observations.observations()) {

        auto t_map_marker = estimate_marker_pose_f_map(observation, camera_pose_f_map,
                                                       camera_matrix, dist_coeffs);

        // Update an existing marker or add a new one.
        auto marker_pair = map().markers().find(observation.id());
        if (marker_pair != map().markers().end()) {
          auto &marker = marker_pair->second;
          marker.update_simple_average(t_map_marker);

        } else {
          map().markers()[observation.id()] = Marker(observation.id(), t_map_marker);
          marker_added = true;
        }
      }

      // For now publish a new map based on the number of images processed after a new marker is added.
      if (marker_added) {
        observations_before_publish = observations_before_publish_base;
        observations_before_publish_reset = observations_before_publish_base;
      } else if (observations_before_publish <= 0) {
        observations_before_publish_reset += observations_before_publish_reset;
        observations_before_publish = observations_before_publish_reset;
      }
      observations_before_publish -= 1;
      return observations_before_publish == 0;
    }

    TransformWithCovariance estimate_marker_pose_f_map(Observation &observation,
                                                       const TransformWithCovariance &camera_pose_f_map,
                                                       const cv::Mat &camera_matrix,
                                                       const cv::Mat &dist_coeffs)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_map = Marker::corners_f_marker(map().marker_length());
      std::vector<cv::Point2f> all_corners_f_image = observation.corners_f_image();

      // Figure out image location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(all_corners_f_map, all_corners_f_image, camera_matrix, dist_coeffs, rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker. This function returns marker_pose_f_map
      // or equivalently t_map_marker. pre-multiply the rvec, tvec transform by camera_pose_f_map
      // before returning it. In other words:
      // t_map_marker = t_map_camera * t_camera_marker.
      auto t_map_marker = camera_pose_f_map.transform() * tf2_util::to_tf2_transform(rvec, tvec);

      // ToDo: get some covariance estimate
      return TransformWithCovariance(t_map_marker, 0.0);
    }


  };

//=============
// VmapNode class
//=============

  class VmapNode : public rclcpp::Node
  {
    Map map_;
    Localizer localizer_;
    std::shared_ptr<Mapper> mapper_;

    int callbacks_processed_{0};

    rclcpp::Subscription<flock_vlam_msgs::msg::Observations>::SharedPtr observations_sub_;

    rclcpp::Publisher<flock_vlam_msgs::msg::Map>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_markers_pub_;

  public:

    explicit VmapNode()
      : Node("vmap_node"), map_(*this), localizer_(*this, map_)
    {
      mapper_.reset(new MapperSimpleAverage(*this, map_));

      // ROS subscriptions
      auto observations_sub_cb = std::bind(&VmapNode::observations_callback, this, std::placeholders::_1);
      observations_sub_ = create_subscription<flock_vlam_msgs::msg::Observations>("/flock_observations",
                                                                                  observations_sub_cb);

      // ROS publishers
      map_pub_ = create_publisher<flock_vlam_msgs::msg::Map>("/flock_map", 8);
      rviz_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("rviz_markers", 1);

      RCLCPP_INFO(get_logger(), "vmap_node ready");
    }

  private:

    void observations_callback(const flock_vlam_msgs::msg::Observations::SharedPtr msg)
    {
      callbacks_processed_ += 1;

      // Get the cameraInfo from the message
      cv::Mat camera_matrix;
      cv::Mat dist_coeffs;
      tf2_util::load_camera_info(msg->camera_info, camera_matrix, dist_coeffs);

      // Get observations from the message.
      Observations observations(*msg);

      // Estimate the camera pose using the latest map estimate
      auto camera_pose_f_map = localizer_.average_camera_pose_f_map(observations, camera_matrix, dist_coeffs);

      // We get an invalid pose if none of the visible markers pose's are known.
      if (camera_pose_f_map.is_valid()) {

        // Update our map with the observations
        auto doPub = mapper_->update_map(camera_pose_f_map, observations, camera_matrix, dist_coeffs);

        // Publish the new map if requested
        if (doPub) {
          auto map_msg = map_.to_map_msg(msg->header, map_.marker_length());
          map_pub_->publish(map_msg);

          // Publish the marker Visualization
          publish_map_visualization();

          // Save the map to a file as well
          map_.save_to_file("src/flock_vlam/flock_vlam/cfg/generated_map.yaml");
        }
      }
    }

    void publish_map_visualization(void) {
      if (count_subscribers(rviz_markers_pub_->get_topic_name()) > 0) {
        visualization_msgs::msg::MarkerArray marker_array_msg;
        for (auto marker_pair: map_.markers()) {
          auto marker = marker_pair.second;
          visualization_msgs::msg::Marker marker_msg;
          marker_msg.id = marker.id();
          marker_msg.header.frame_id = "map";
          marker_msg.pose = marker.t_map_marker().to_pose_msg();
          marker_msg.type = marker_msg.CUBE;
          marker_msg.action = marker_msg.ADD;
          marker_msg.scale.x = 0.1;
          marker_msg.scale.y = 0.1;
          marker_msg.scale.z = 0.01;
          marker_msg.color.r = 1.f;
          marker_msg.color.g = 1.f;
          marker_msg.color.b = 0.f;
          marker_msg.color.a = 1.f;
          marker_array_msg.markers.push_back(marker_msg);
        }
        rviz_markers_pub_->publish(marker_array_msg);
      }
    }
  };
}

//=============
// main()
//=============

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<flock_vlam::VmapNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
