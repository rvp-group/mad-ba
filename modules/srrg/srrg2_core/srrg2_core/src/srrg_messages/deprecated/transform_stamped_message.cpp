#include "transform_stamped_message.h"

namespace srrg2_core {

  TransformStampedMessage::TransformStampedMessage(const std::string& topic_,
                           const std::string& frame_id_,
                           const int& seq_,
                           const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(to_frame_id, ""),
    SETUP_PROPERTY(pose, Isometry3f::Identity()) {
  }

  BOSS_REGISTER_CLASS(TransformStampedMessage);

}  // namespace srrg2_core
