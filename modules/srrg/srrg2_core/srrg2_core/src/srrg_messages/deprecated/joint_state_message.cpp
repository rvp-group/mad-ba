#include "joint_state_message.h"

namespace srrg2_core {

  JointStateMessage::JointStateMessage(const std::string& topic_,
                                       const std::string& frame_id_,
                                       const int& seq_,
                                       const double& timestamp_): BaseSensorMessage(topic_, frame_id_, seq_, timestamp_) {}

  JointStateMessage::~JointStateMessage() {}

  BOSS_REGISTER_CLASS(JointStateMessage);

} //namespace srrg2_core
