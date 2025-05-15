#include "cmd_vel_message.h"

namespace srrg2_core {
  CmdVelMessage::CmdVelMessage(const std::string& topic_,
                               const std::string& frame_id_,
                               const int& seq_,
                               const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(linear, 0),
    SETUP_PROPERTY(angular, 0) {
  }

} // namespace srrg2_core
