
#include "map.hpp"

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

  }

  flock_vlam_msgs::msg::Observations Observations::to_msg(geometry_msgs::msg::PoseWithCovarianceStamped & t_map_camera)
  {
    flock_vlam_msgs::msg::Observations msg;
    return msg;
  }

  Map::Map(rclcpp::Node & node)
  : node_(node), markers_()
  {
  }

  void Map::load_from_msg(const flock_vlam_msgs::msg::Map::SharedPtr msg)
  {
  }

  TransformWithCovariance Map::estimate_t_map_camera(Observations & observations)
  {
    return TransformWithCovariance();
  }
}
