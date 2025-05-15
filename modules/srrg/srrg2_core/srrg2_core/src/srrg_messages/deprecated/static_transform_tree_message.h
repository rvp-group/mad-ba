#pragma once
#include "srrg_property/property_serializable.h"
#include "base_sensor_message.h"
#include "static_transform_tree.h"

namespace srrg2_core {

  class StaticTransformTreeMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StaticTransformTreeMessage(const std::string& topic_ = "",
                               const std::string& frame_id_ = "",
                               const int& seq_ = -1,
                               const double& timestamp_ = -1);
    PropertySerializable_<StaticTransformTree> tree;
  };

} /* namespace srrg2_core */

