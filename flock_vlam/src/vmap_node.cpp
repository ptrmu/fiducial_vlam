

#include "rclcpp/rclcpp.hpp"

#include "flock_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

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

    auto &node()
    { return node_; }

    auto &map()
    { return map_; }

    virtual bool
    update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations, float marker_length,
               const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) = 0;
  };

//=============
// MapperSimpleAverage class
//=============

  class MapperSimpleAverage : public Mapper
  {
    int observations_processed_{0};

  public:
    MapperSimpleAverage(rclcpp::Node &node, Map &map)
      : Mapper(node, map)
    {
    }

    virtual ~MapperSimpleAverage()
    {}

    virtual bool
    update_map(const TransformWithCovariance &camera_pose_f_map, Observations &observations, float marker_length,
               const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
    {
      RCLCPP_INFO(node().get_logger(), "Processing %d observations", observations.observations().size());

      // For all observations estimate the marker location and update the map
      for (auto observation : observations.observations()) {

        auto t_map_marker = estimate_marker_pose_f_map(observation, camera_pose_f_map,
                                                       marker_length, camera_matrix, dist_coeffs);

        RCLCPP_INFO(node().get_logger(), "update marker %d", observation.id());

        RCLCPP_INFO(node().get_logger(), "camera xyz:%lf %lf %lf",
                    camera_pose_f_map.transform().getOrigin().x(),
                    camera_pose_f_map.transform().getOrigin().y(),
                    camera_pose_f_map.transform().getOrigin().z());
        RCLCPP_INFO(node().get_logger(), "new xyz:%lf %lf %lf",
                    t_map_marker.transform().getOrigin().x(),
                    t_map_marker.transform().getOrigin().y(),
                    t_map_marker.transform().getOrigin().z());


        // Update an existing marker or add a new one.
        auto marker_pair = map().markers().find(observation.id());
        if (marker_pair != map().markers().end()) {
          auto &marker = marker_pair->second;
          RCLCPP_INFO(node().get_logger(), "old %d xyz:%lf %lf %lf",
                      marker.update_count(),
                      marker.marker_pose_f_map().transform().getOrigin().x(),
                      marker.marker_pose_f_map().transform().getOrigin().y(),
                      marker.marker_pose_f_map().transform().getOrigin().z());
          marker.update_simple_average(t_map_marker);
        } else {
          RCLCPP_INFO(node().get_logger(), "old doesn't exist");
          map().markers()[observation.id()] = Marker(observation.id(), t_map_marker);
        }
      }

      // for now publish a new map for every 10 images processed.
      return ((++observations_processed_) % 10) == 0;
    }

    TransformWithCovariance estimate_marker_pose_f_map(Observation &observation,
                                                       const TransformWithCovariance &camera_pose_f_map,
                                                       float marker_length, const cv::Mat &camera_matrix,
                                                       const cv::Mat &dist_coeffs)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_map = Marker::corners_f_marker(marker_length);
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

    float marker_length_{0.18415};

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

      RCLCPP_INFO(get_logger(), "vmap_node ready");
    }

  private:

    void observations_callback(const flock_vlam_msgs::msg::Observations::SharedPtr msg)
    {

      // Get the cameraInfo from the message
      cv::Mat camera_matrix;
      cv::Mat dist_coeffs;
      tf2_util::load_camera_info(msg->camera_info, camera_matrix, dist_coeffs);

      // Get observations from the message.
      Observations observations(*msg);

      // Estimate the camera pose using the latest map estimate
      auto camera_pose_f_map = localizer_.estimate_camera_pose_f_map(observations, marker_length_, camera_matrix,
                                                                     dist_coeffs);

      // Update our map with the observations
      auto doPub = mapper_->update_map(camera_pose_f_map, observations, marker_length_,
                                       camera_matrix, dist_coeffs);

      // Publish the new map if requested
      if (doPub) {
        // auto map_msg = map_.to_map_msg(msg->header);
      }
    }

    rclcpp::Subscription<flock_vlam_msgs::msg::Observations>::SharedPtr observations_sub_;

    rclcpp::Publisher<flock_vlam_msgs::msg::Map>::SharedPtr map_pub_;
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
