#pragma once

#include "base_sensor_message.h"
#include <srrg_property/property_eigen.h>

namespace srrg2_core {
  class OdometryMessage : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OdometryMessage(const std::string& topic_    = "",
                    const std::string& frame_id_ = "",
                    const int& seq_              = -1,
                    const double& timestamp_     = -1);

    PropertyString child_frame;
    PropertyEigen_<Isometry3f> pose;
    PropertyEigen_<Vector3f> linear_velocity;
    PropertyEigen_<Vector3f> angular_velocity;
    PropertyEigen_<Matrix6f> pose_covariance;
    PropertyEigen_<Matrix6f> twist_covariance;
  };

  using OdometryMessagePtr = std::shared_ptr<OdometryMessage>;

} // namespace srrg2_core
