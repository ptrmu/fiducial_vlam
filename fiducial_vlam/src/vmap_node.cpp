
#include <chrono>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "map.hpp"
#include "vmap_context.hpp"
#include "fiducial_math.hpp"


namespace fiducial_vlam
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
               const CameraInfo &ci) = 0;
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
               const CameraInfo &ci)
    {
      FiducialMath fm{ci, map().marker_length()};

      bool marker_added{false};

      // For all observations estimate the marker location and update the map
      for (auto observation : observations.observations()) {

        auto t_map_marker = fm.solve_t_map_marker(observation, camera_pose_f_map);

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

  };

//=============
// VmapNode class
//=============

  class VmapNode : public rclcpp::Node
  {
    VmapContext cxt_;
    Map map_;
    Localizer localizer_;
    std::shared_ptr<Mapper> mapper_;

    int callbacks_processed_{0};

    // ROS publishers
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Map>::SharedPtr map_pub_ =
      create_publisher<fiducial_vlam_msgs::msg::Map>("fiducial_map", 16);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("rviz_markers", 16);
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_ =
      create_publisher<tf2_msgs::msg::TFMessage>("tf", 16);

    rclcpp::Subscription<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_sub_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    bool map_published_recently_ = false;

  public:

    VmapNode()
      : Node("vmap_node"), cxt_(), map_(*this), localizer_(*this, map_)
    {
      mapper_ = std::make_shared<MapperSimpleAverage>(*this, map_);

      // Get parameters from the command line
      cxt_.load_parameters(*this);

      // ROS subscriptions
      observations_sub_ = create_subscription<fiducial_vlam_msgs::msg::Observations>(
        "fiducial_observations",
        [this](const fiducial_vlam_msgs::msg::Observations::UniquePtr msg) -> void
        {
          this->observations_callback(msg);
        },
        16);

      // Timer for publishing map info
      map_pub_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000. / cxt_.marker_map_publish_frequency_hz_)),
        [this]() -> void
        {
          if (!this->map_published_recently_) {
            this->publish_map_and_visualization();
          }
          this->map_published_recently_ = false;
        });

      RCLCPP_INFO(get_logger(), "vmap_node ready");
    }

  private:

    void observations_callback(const fiducial_vlam_msgs::msg::Observations::UniquePtr &msg)
    {
      callbacks_processed_ += 1;

      CameraInfo ci{msg->camera_info};

      // Get the cameraInfo from the message
      cv::Mat camera_matrix;
      cv::Mat dist_coeffs;
      to_camera_info(msg->camera_info, camera_matrix, dist_coeffs);

      // Get observations from the message.
      Observations observations(*msg);

      // Estimate the camera pose using the latest map estimate
      auto camera_pose_f_map = localizer_.average_camera_pose_f_map(observations, camera_matrix, dist_coeffs);

      // We get an invalid pose if none of the visible markers pose's are known.
      if (camera_pose_f_map.is_valid()) {

        // Update our map with the observations
        auto doPub = mapper_->update_map(camera_pose_f_map, observations, ci);

        // Publish the new map if requested
        if (doPub) {
          publish_map_and_visualization();

          // Save the map to a file as well
          //map_.save_to_file("src/fiducial_vlam/fiducial_vlam/cfg/generated_map.yaml");
        }
      }
    }

    tf2_msgs::msg::TFMessage to_tf_message()
    {
      auto stamp = now();
      tf2_msgs::msg::TFMessage tf_message;

      for (auto &marker_pair: map_.markers()) {
        auto &marker = marker_pair.second;
        auto mu = marker.t_map_marker().mu();

        std::ostringstream oss_child_frame_id;
        oss_child_frame_id << "marker_" << std::setfill('0') << std::setw(3) << marker.id();

        tf2::Quaternion q;
        q.setRPY(mu[3], mu[4], mu[5]);
        auto tf2_transform = tf2::Transform(q, tf2::Vector3(mu[0], mu[1], mu[2]));

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = oss_child_frame_id.str();
        msg.transform = tf2::toMsg(tf2_transform);

        tf_message.transforms.emplace_back(msg);
      }

      return tf_message;
    }

    visualization_msgs::msg::MarkerArray to_marker_array_msg()
    {
      visualization_msgs::msg::MarkerArray markers;
      for (auto &marker_pair: map_.markers()) {
        auto &marker = marker_pair.second;
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.id = marker.id();
        marker_msg.header.frame_id = "map";
        marker_msg.pose = to_Pose_msg(marker.t_map_marker());
        marker_msg.type = marker_msg.CUBE;
        marker_msg.action = marker_msg.ADD;
        marker_msg.scale.x = 0.1;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.01;
        marker_msg.color.r = 1.f;
        marker_msg.color.g = 1.f;
        marker_msg.color.b = 0.f;
        marker_msg.color.a = 1.f;
        markers.markers.emplace_back(marker_msg);
      }
      return markers;
    }

    void publish_map_and_visualization()
    {
      // publish the map
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = "map";
      map_pub_->publish(map_.to_map_msg(header, map_.marker_length()));

      // Publish the marker Visualization
      if (cxt_.publish_marker_visualizations_) {
        rviz_markers_pub_->publish(to_marker_array_msg());
      }

      // Publish the TFtree
      if (cxt_.publish_marker_tfs_) {
        tf_message_pub_->publish(to_tf_message());
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
  auto node = std::make_shared<fiducial_vlam::VmapNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
