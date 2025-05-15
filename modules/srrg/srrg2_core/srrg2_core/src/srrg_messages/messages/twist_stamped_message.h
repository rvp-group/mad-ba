#pragma once
#include "srrg_property/property_eigen.h"
#include "base_sensor_message.h"

namespace srrg2_core {

  class TwistStampedMessage: public BaseSensorMessage  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TwistStampedMessage(const std::string& topic_ = "",
        const std::string& frame_id_ = "",
        const int& seq_ = -1,
        const double& timestamp_ = -1);
    PropertyEigen_<Vector3f> linear;
    PropertyEigen_<Vector3f> angular;
  };
  using TwistStampedMessagePtr = std::shared_ptr<TwistStampedMessage>;

} /* namespace srrg2_slam_architecture */
