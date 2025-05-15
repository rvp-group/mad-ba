#pragma once
#include "srrg_property/property_eigen.h"
#include "base_sensor_message.h"

namespace srrg2_core {

  class PointStampedMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointStampedMessage(const std::string& topic_ = "",
                 const std::string& frame_id_ = "",
                 const int& seq_ = -1,
                 const double& timestamp_ = -1);
    PropertyEigen_<Vector3f> point;
  };

  using PointStampedMessagePtr = std::shared_ptr<PointStampedMessage>;

} /* namespace srrg2_slam_architecture */
