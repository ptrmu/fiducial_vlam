
#include "fiducial_math.hpp"

#include "marker.hpp"
#include "convert_util.hpp"

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
// FiducialMath::CvFiducialMath class
// ==============================================================================

  class FiducialMath::CvFiducialMath
  {
    const CameraInfo ci_;
    const double marker_length_;

  public:
    CvFiducialMath(const CameraInfo &camera_info, double marker_length)
      : ci_(camera_info), marker_length_(marker_length)
    {}

    CvFiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg, double marker_length)
      : ci_(camera_info_msg), marker_length_(marker_length)
    {}

    TransformWithCovariance solve_t_map_marker(
      const Observation &observation,
      const TransformWithCovariance &t_map_camera)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_marker = corners_f_marker();
      std::vector<cv::Point2f> all_corners_f_image = observation.corners_f_image();

      // Figure out image location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(all_corners_f_marker, all_corners_f_image,
                   ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker. This function returns marker_pose_f_map
      // or equivalently t_map_marker. pre-multiply the rvec, tvec transform by t_map_camera
      // before returning it. In other words:
      // t_map_marker = t_map_camera * t_camera_marker.
      auto t_map_marker = t_map_camera.transform() * to_tf2_transform(rvec, tvec);

      // ToDo: get some covariance estimate
      return TransformWithCovariance(t_map_marker);
    }

  private:
    std::vector<cv::Point3d> corners_f_map(const Marker &marker)
    {
      // Build up a list of the corner locations in the map frame.
      tf2::Vector3 corner0_f_marker(-marker_length_ / 2.f, marker_length_ / 2.f, 0.f);
      tf2::Vector3 corner1_f_marker(marker_length_ / 2.f, marker_length_ / 2.f, 0.f);
      tf2::Vector3 corner2_f_marker(marker_length_ / 2.f, -marker_length_ / 2.f, 0.f);
      tf2::Vector3 corner3_f_marker(-marker_length_ / 2.f, -marker_length_ / 2.f, 0.f);

      auto t_map_marker_tf = marker.t_map_marker().transform();
      auto corner0_f_map = t_map_marker_tf * corner0_f_marker;
      auto corner1_f_map = t_map_marker_tf * corner1_f_marker;
      auto corner2_f_map = t_map_marker_tf * corner2_f_marker;
      auto corner3_f_map = t_map_marker_tf * corner3_f_marker;

      std::vector<cv::Point3d> corners_f_map;
      corners_f_map.emplace_back(cv::Point3d(corner0_f_map.x(), corner0_f_map.y(), corner0_f_map.z()));
      corners_f_map.emplace_back(cv::Point3d(corner1_f_map.x(), corner1_f_map.y(), corner1_f_map.z()));
      corners_f_map.emplace_back(cv::Point3d(corner2_f_map.x(), corner2_f_map.y(), corner2_f_map.z()));
      corners_f_map.emplace_back(cv::Point3d(corner3_f_map.x(), corner3_f_map.y(), corner3_f_map.z()));

      return corners_f_map;
    }

    std::vector<cv::Point3d> corners_f_marker()
    {
      // Build up a list of the corner locations in the map frame.
      std::vector<cv::Point3d> corners_f_marker;
      corners_f_marker.emplace_back(cv::Point3d(-marker_length_ / 2.f, marker_length_ / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length_ / 2.f, marker_length_ / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length_ / 2.f, -marker_length_ / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(-marker_length_ / 2.f, -marker_length_ / 2.f, 0.f));
      return corners_f_marker;
    }

    tf2::Transform to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
    {
      tf2::Vector3 t(tvec[0], tvec[1], tvec[2]);
      cv::Mat rmat;
      cv::Rodrigues(rvec, rmat);
      tf2::Matrix3x3 m;
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          m[row][col] = rmat.at<double>(row, col);  // Row- vs. column-major order
        }
      }
      tf2::Transform result(m, t);
      return result;
    }
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  FiducialMath::FiducialMath(const CameraInfo &camera_info, double marker_length)
    : cv_(std::make_shared<CvFiducialMath>(camera_info, marker_length))
  {}

  FiducialMath::FiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg, double marker_length)
    : cv_(std::make_shared<CvFiducialMath>(camera_info_msg, marker_length))
  {}

  TransformWithCovariance FiducialMath::solve_t_map_marker(
    const Observation &observation,
    const TransformWithCovariance &t_map_camera)
  {
    return cv_->solve_t_map_marker(observation, t_map_camera);
  }
}