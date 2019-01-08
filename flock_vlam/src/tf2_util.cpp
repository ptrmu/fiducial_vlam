

#include "tf2_util.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "opencv2/calib3d/calib3d.hpp"

namespace tf2_util
{

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
}

