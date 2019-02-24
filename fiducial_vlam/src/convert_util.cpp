

#include "convert_util.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

namespace fiducial_vlam
{
  geometry_msgs::msg::Pose to_Pose_msg(const TransformWithCovariance &twc)
  {
    geometry_msgs::msg::Pose pose;
    toMsg(twc.transform(), pose);
    return pose;
  }

  geometry_msgs::msg::PoseWithCovariance to_PoseWithCovariance_msg(const TransformWithCovariance &twc)
  {
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.pose = to_Pose_msg(twc);
    //msg.set__covariance() // ToDo move over the covariance
    return msg;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped to_PoseWithCovarianceStamped_msg(
    const TransformWithCovariance &twc, const std_msgs::msg::Header &header)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header = header;
    msg.pose = to_PoseWithCovariance_msg(twc);
    return msg;
  }

  TransformWithCovariance to_TransformWithCovariance(const geometry_msgs::msg::PoseWithCovariance &pwc)
  {
    tf2::Transform tf;
    fromMsg(pwc.pose, tf);
    return TransformWithCovariance(tf);
  }

  // todo remove when finished refactor
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

  void to_cv_rvec_tvec(const tf2::Transform &transform, cv::Vec3d &rvec, cv::Vec3d &tvec)
  {
    auto t = transform.getOrigin();
    tvec[0] = t.x();
    tvec[1] = t.y();
    tvec[2] = t.z();
    auto R = transform.getBasis();
    cv::Mat rmat(3, 3, CV_64FC1);
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        rmat.at<double>(row, col) = R[row][col];
      }
    }
    cv::Rodrigues(rmat, rvec);
  }

  void to_camera_info(const sensor_msgs::msg::CameraInfo &msg, cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
  {
    camera_matrix = cv::Mat(3, 3, CV_64F, 0.);
    camera_matrix.at<double>(0, 0) = msg.k[0];
    camera_matrix.at<double>(0, 2) = msg.k[2];
    camera_matrix.at<double>(1, 1) = msg.k[4];
    camera_matrix.at<double>(1, 2) = msg.k[5];
    camera_matrix.at<double>(2, 2) = 1.;

    // ROS and OpenCV (and everybody?) agree on this ordering: k1, k2, t1 (p1), t2 (p2), k3
    dist_coeffs = cv::Mat(1, 5, CV_64F);
    dist_coeffs.at<double>(0) = msg.d[0];
    dist_coeffs.at<double>(1) = msg.d[1];
    dist_coeffs.at<double>(2) = msg.d[2];
    dist_coeffs.at<double>(3) = msg.d[3];
    dist_coeffs.at<double>(4) = msg.d[4];
  }
}

