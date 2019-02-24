
#include "fiducial_math.hpp"

#include "opencv2/calib3d/calib3d.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// CameraInfo::CvCameraInfo class
// ==============================================================================

  class CameraInfo::CvCameraInfo
  {
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

  public:
    CvCameraInfo() = delete;

    explicit CvCameraInfo(const sensor_msgs::msg::CameraInfo &msg)
      : camera_matrix_(3, 3, CV_64F, 0.), dist_coeffs_(1, 5, CV_64F)
    {
      camera_matrix_.at<double>(0, 0) = msg.k[0];
      camera_matrix_.at<double>(0, 2) = msg.k[2];
      camera_matrix_.at<double>(1, 1) = msg.k[4];
      camera_matrix_.at<double>(1, 2) = msg.k[5];
      camera_matrix_.at<double>(2, 2) = 1.;

      // ROS and OpenCV (and everybody?) agree on this ordering: k1, k2, t1 (p1), t2 (p2), k3
      dist_coeffs_.at<double>(0) = msg.d[0];
      dist_coeffs_.at<double>(1) = msg.d[1];
      dist_coeffs_.at<double>(2) = msg.d[2];
      dist_coeffs_.at<double>(3) = msg.d[3];
      dist_coeffs_.at<double>(4) = msg.d[4];
    }

    auto &camera_matrix()
    { return camera_matrix_; }

    auto &dist_coeffs()
    { return dist_coeffs_; }
  };

// ==============================================================================
// CameraInfo class
// ==============================================================================

  CameraInfo::CameraInfo(const sensor_msgs::msg::CameraInfo &camera_info_msg)
    : cv_(std::make_shared<CameraInfo::CvCameraInfo>(camera_info_msg))
  {}

// ==============================================================================
// FiducialMath class
// ==============================================================================

  TransformWithCovariance FiducialMath::estimate_t_map_marker(
    const Observation &observation,
    const TransformWithCovariance &camera_pose_f_map)
  {
    // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
    std::vector<cv::Point3d> all_corners_f_marker = Marker::corners_f_marker(marker_length_);
    std::vector<cv::Point2f> all_corners_f_image = observation.corners_f_image();

    // Figure out image location.
    cv::Vec3d rvec, tvec;
    cv::solvePnP(all_corners_f_marker, all_corners_f_image,
                 ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
                 rvec, tvec);

    // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
    // camera coordinate system". In this case the marker frame is the model coordinate system.
    // So rvec, tvec are the transformation t_camera_marker. This function returns marker_pose_f_map
    // or equivalently t_map_marker. pre-multiply the rvec, tvec transform by camera_pose_f_map
    // before returning it. In other words:
    // t_map_marker = t_map_camera * t_camera_marker.
    auto t_map_marker = camera_pose_f_map.transform() * to_tf2_transform(rvec, tvec);

    // ToDo: get some covariance estimate
    return TransformWithCovariance(t_map_marker);

  }

}