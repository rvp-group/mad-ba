#pragma once
#include "srrg_property/property_eigen.h"
#include "base_sensor_message.h"

namespace srrg2_core {

  using namespace geometry3d;

  class TransformStampedMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TransformStampedMessage(const std::string& topic_ = "",
                const std::string& frame_id_ = "",
                const int& seq_ = -1,
                const double& timestamp_ = -1);

    PropertyString to_frame_id;
    PropertyEigen_<Isometry3f> pose;
  };

}  // namespace srrg2_core
