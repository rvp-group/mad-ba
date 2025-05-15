#pragma once
#include "base_sensor_message.h"
#include "srrg_property/property_vector.h"

namespace srrg2_core {

  enum RadiationType {
    ULTRASOUND = 0, INFRARED = 1
  };

  class RangeMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RangeMessage(const std::string& topic_ = "",
                 const std::string& frame_id_ = "",
                 const int& seq_ = -1,
                 const double& timestamp_ = -1.0);
    PropertyFloat range_min;
    PropertyFloat range_max;
    PropertyFloat field_of_view;
    PropertyInt radiation_type;
    PropertyFloat range;
  };

  using RangeMessagePtr = std::shared_ptr<RangeMessage>;
} //namespace srrg2_core
