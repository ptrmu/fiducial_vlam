
#include <chrono>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "fiducial_math.hpp"
#include "map.hpp"
#include "vmap_context.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


namespace fiducial_vlam
{

// ==============================================================================
// Mapper class
// ==============================================================================

  class Mapper
  {
    std::shared_ptr<Map> &map_;

  public:
    explicit Mapper(std::shared_ptr<Map> &map)
      : map_(map)
    {
    }

    Mapper() = delete;

    virtual ~Mapper() = default;

    auto &map()
    { return map_; }

    virtual void
    update_map(const TransformWithCovariance &camera_pose_f_map,
               const Observations &observations,
               FiducialMath &fm) = 0;
  };

// ==============================================================================
// MapperSimpleAverage class
// ==============================================================================

  class MapperSimpleAverage : public Mapper
  {
  public:
    explicit MapperSimpleAverage(std::shared_ptr<Map> &map)
      : Mapper(map)
    {
    }

    void
    update_map(const TransformWithCovariance &t_map_camera,
               const Observations &observations,
               FiducialMath &fm) override
    {
      // For all observations estimate the marker location and update the map
      for (auto &observation : observations.observations()) {

        auto t_camera_marker = fm.solve_t_camera_marker(observation, map()->marker_length());
        auto t_map_marker = TransformWithCovariance(t_map_camera.transform() * t_camera_marker.transform());

        // Update an existing marker or add a new one.
        auto iter = map()->markers().find(observation.id());
        if (iter != map()->markers().end()) {
          auto &marker = iter->second;
          update_marker_simple_average(marker, t_map_marker);

        } else {
          map()->markers()[observation.id()] = Marker(observation.id(), t_map_marker);
        }
      }
    }

    void update_marker_simple_average(Marker &existing, const TransformWithCovariance &another_twc)
    {
      if (!existing.is_fixed()) {
        auto t_map_marker = existing.t_map_marker();  // Make a copy
        auto update_count = existing.update_count();
        t_map_marker.update_simple_average(another_twc, update_count);
        existing.set_t_map_marker(t_map_marker);
        existing.set_update_count(update_count+1);
      }
    }
  };

// ==============================================================================
// VmapNode class
// ==============================================================================

  class VmapNode : public rclcpp::Node
  {
    VmapContext cxt_{};
    std::shared_ptr<Map> map_ = std::make_shared<Map>();
    Localizer localizer_{map_};
    std::shared_ptr<Mapper> mapper_{};

    int callbacks_processed_{0};

    // ROS publishers
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Map>::SharedPtr map_pub_ =
      create_publisher<fiducial_vlam_msgs::msg::Map>("fiducial_map", 16);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fiducial_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("fiducial_markers", 16);
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_ =
      create_publisher<tf2_msgs::msg::TFMessage>("tf", 16);

    rclcpp::Subscription<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_sub_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

  public:

    VmapNode()
      : Node("vmap_node")
    {
      // Get parameters from the command line
      cxt_.load_parameters(*this);

      // construct a map builder.
      mapper_ = std::make_shared<MapperSimpleAverage>(map_);

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
          this->publish_map_and_visualization();
        });

      RCLCPP_INFO(get_logger(), "vmap_node ready");
    }

  private:

    void observations_callback(const fiducial_vlam_msgs::msg::Observations::UniquePtr &msg)
    {
      callbacks_processed_ += 1;

      CameraInfo ci{msg->camera_info};
      FiducialMath fm{ci};

      // Get observations from the message.
      Observations observations(*msg);

      // THere is nothing to do at this point unless we have more than one observation.
      if (observations.size() < 2) {
        return;
      }

      // Estimate the camera pose using the latest map estimate
      auto t_map_camera = localizer_.average_t_map_camera(observations, fm);

      // We get an invalid pose if none of the visible markers pose's are known.
      if (t_map_camera.is_valid()) {

        // Update our map with the observations
        mapper_->update_map(t_map_camera, observations, fm);
      }
    }

    tf2_msgs::msg::TFMessage to_tf_message()
    {
      auto stamp = now();
      tf2_msgs::msg::TFMessage tf_message;

      for (auto &marker_pair: map_->markers()) {
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
      for (auto &marker_pair: map_->markers()) {
        auto &marker = marker_pair.second;
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.id = marker.id();
        marker_msg.header.frame_id = "map";
        marker_msg.pose = to_Pose_msg(marker.t_map_marker());
        marker_msg.type = visualization_msgs::msg::Marker::CUBE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
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
      map_pub_->publish(map_->to_map_msg(header, map_->marker_length()));

      // Publish the marker Visualization
      if (cxt_.publish_marker_visualizations_) {
        fiducial_markers_pub_->publish(to_marker_array_msg());
      }

      // Publish the transform tree
      if (cxt_.publish_marker_tfs_) {
        tf_message_pub_->publish(to_tf_message());
      }
    }
  };
}

// ==============================================================================
// main()
// ==============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

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
