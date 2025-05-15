#pragma once
#include "base_sensor_message.h"
#include "srrg_data_structures/events.h"

namespace srrg2_core {

  class JointsMessage: public BaseSensorMessage {
  public:
    JointsMessage(const std::string& topic_ = "",
                  const std::string& frame_id_ = "",
                  const int& seq_ = -1,
                  const double& timestamp_ = -1);

    PropertyJointEventVector joint_events;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using JointsMessagePtr = std::shared_ptr<JointsMessage>;

}
