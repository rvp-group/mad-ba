#pragma once
#include "../srrg_data_structures/events.h"
#include "srrg_property/property_vector.h"
#include "base_sensor_message.h"

namespace srrg2_core {

  class JointStateMessage: public BaseSensorMessage {
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    JointStateMessage(const std::string& topic_ = "",
                      const std::string& frame_id_ = "",
                      const int& seq_ = -1,
                      const double& timestamp_ = -1.0);
    ~JointStateMessage();

    BaseEventPtrVector joint_events; //ds TODO serialization
  };

} //namespace srrg2_core
