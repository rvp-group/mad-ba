#pragma once
#include "message_sink_base.h"
#include "srrg_config/configurable.h"
#include "srrg_config/param_macros.h"

namespace srrg2_core {

  class MessageOdomSubsamplerSink : public MessageSinkBase {
  public:
    PARAM(PropertyString, odom_topic, "odometry topic to subsample", "/odom", 0);
    PARAM(PropertyFloat, translation_min, "minimum translation between odometries [m]", 0.25, 0);
    PARAM(PropertyFloat, rotation_min, "minimum rotation between odometries [rad]", 0.25, 0);
    bool putMessage(BaseSensorMessagePtr msg_) override;
    void reset() override;

  protected:
    float _cum_rotation    = 0; // cumulative rotation since last output message
    float _cum_translation = 0; // cumulative translation since last output message
    Isometry3f _prev_pose  = Isometry3f::Identity();
  };

} // namespace srrg2_core
