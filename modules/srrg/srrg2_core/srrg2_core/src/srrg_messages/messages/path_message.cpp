#include "path_message.h"

namespace srrg2_core {
  PathMessage::PathMessage(const std::string& topic_,
                           const std::string& frame_id_,
                           const int& seq_,
                           const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY_NV(poses)
  {}

  PathMessage::~PathMessage(){}

}
