
#include "fiducial_math.hpp"

#include "marker.hpp"
#include "convert_util.hpp"

#include "opencv2/aruco.hpp"
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

  CameraInfo::CameraInfo()
  {}

  CameraInfo::CameraInfo(const sensor_msgs::msg::CameraInfo &camera_info_msg)
    : cv_(std::make_shared<CameraInfo::CvCameraInfo>(camera_info_msg))
  {}

// ==============================================================================
// FiducialMath::CvFiducialMath class
// ==============================================================================

  class FiducialMath::CvFiducialMath
  {
    const CameraInfo ci_;

  public:
    CvFiducialMath(const CameraInfo &camera_info)
      : ci_(camera_info)
    {}

    CvFiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg)
      : ci_(camera_info_msg)
    {}

    TransformWithCovariance solve_t_camera_marker(
      const Observation &observation,
      double marker_length)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_marker = corners_f_marker(marker_length);
      std::vector<cv::Point2f> all_corners_f_image = corners_f_image(observation);

      // Figure out image location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(all_corners_f_marker, all_corners_f_image,
                   ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker.
      return TransformWithCovariance(to_tf2_transform(rvec, tvec));
    }

    Observations detect_markers(cv_bridge::CvImagePtr &color)
    {
      // Todo: make the dictionary a parameter
      auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      auto detectorParameters = cv::aruco::DetectorParameters::create();
      detectorParameters->doCornerRefinement = true;

      // Color to gray for detection
      cv::Mat gray;
      cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary, corners, ids, detectorParameters);

      // return the corners as a list of observations
      return to_observations(ids, corners);
    }

    void annotate_image_with_marker_axis(cv_bridge::CvImagePtr &color, const TransformWithCovariance &t_camera_marker)
    {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

      cv::aruco::drawAxis(color->image,
                          ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
                          rvec, tvec, 0.1);
    }

  private:
    std::vector<cv::Point3d> corners_f_map(const Marker &marker, double marker_length)
    {
      // Build up a list of the corner locations in the map frame.
      tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
      tf2::Vector3 corner1_f_marker(marker_length / 2.f, marker_length / 2.f, 0.f);
      tf2::Vector3 corner2_f_marker(marker_length / 2.f, -marker_length / 2.f, 0.f);
      tf2::Vector3 corner3_f_marker(-marker_length / 2.f, -marker_length / 2.f, 0.f);

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

    std::vector<cv::Point3d> corners_f_marker(double marker_length)
    {
      // Build up a list of the corner locations in the map frame.
      std::vector<cv::Point3d> corners_f_marker;
      corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, -marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, -marker_length / 2.f, 0.f));
      return corners_f_marker;
    }

    std::vector<cv::Point2f> corners_f_image(const Observation &observation)
    {
      return std::vector<cv::Point2f>{
        cv::Point2f(observation.x0(), observation.y0()),
        cv::Point2f(observation.x1(), observation.y1()),
        cv::Point2f(observation.x2(), observation.y2()),
        cv::Point2f(observation.x3(), observation.y3())};
    };

    Observations to_observations(const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners)
    {
      Observations observations;
      for (int i = 0; i < ids.size(); i += 1) {
        observations.add(Observation(ids[i],
                                     corners[i][0].x, corners[i][0].y,
                                     corners[i][1].x, corners[i][1].y,
                                     corners[i][2].x, corners[i][2].y,
                                     corners[i][3].x, corners[i][3].y));
      }
      return observations;
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

    void to_cv_rvec_tvec(const TransformWithCovariance &t, cv::Vec3d &rvec, cv::Vec3d &tvec)
    {
      auto c = t.transform().getOrigin();
      tvec[0] = c.x();
      tvec[1] = c.y();
      tvec[2] = c.z();
      auto R = t.transform().getBasis();
      cv::Mat rmat(3, 3, CV_64FC1);
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          rmat.at<double>(row, col) = R[row][col];
        }
      }
      cv::Rodrigues(rmat, rvec);
    }
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  FiducialMath::FiducialMath(const CameraInfo &camera_info)
    : cv_(std::make_shared<CvFiducialMath>(camera_info))
  {}

  FiducialMath::FiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg)
    : cv_(std::make_shared<CvFiducialMath>(camera_info_msg))
  {}

  TransformWithCovariance FiducialMath::solve_t_camera_marker(
    const Observation &observation,
    double marker_length)
  {
    return cv_->solve_t_camera_marker(observation, marker_length);
  }

  Observations FiducialMath::detect_markers(cv_bridge::CvImagePtr &color)
  {
    return cv_->detect_markers(color);
  }

  void FiducialMath::annotate_image_with_marker_axis(cv_bridge::CvImagePtr &color,
                                                     const TransformWithCovariance &t_camera_marker)
  {
    cv_->annotate_image_with_marker_axis(color, t_camera_marker);
  }
}