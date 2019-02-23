
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{
  void TransformWithCovariance::update_simple_average(const TransformWithCovariance &newVal, int previous_update_count)
  {
    double previous_weight = double(previous_update_count) / (previous_update_count + 1);
    double current_weight = 1.0 / (previous_update_count + 1);

    transform_.setOrigin(transform_.getOrigin() * previous_weight +
                         newVal.transform_.getOrigin() * current_weight);

    tf2::Quaternion q1 = transform_.getRotation();
    tf2::Quaternion q2 = newVal.transform_.getRotation();
    transform_.setRotation(q1.slerp(q2, current_weight).normalized());
  }
}

