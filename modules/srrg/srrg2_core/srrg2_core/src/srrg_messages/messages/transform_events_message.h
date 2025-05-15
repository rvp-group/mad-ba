#pragma once
#include "srrg_property/property_eigen.h"
#include "base_sensor_message.h"
#include "srrg_data_structures/events.h"

namespace srrg2_core{

  class TransformEventsMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    TransformEventsMessage(const std::string& topic_ = "",
                           const std::string& frame_id_ = "",
                           const int& seq_ = -1,
                           const double& timestamp_ = -1);

    PropertyTransformEventVector events;
  };

  using TransformEventsMessagePtr = std::shared_ptr<TransformEventsMessage>;

}
