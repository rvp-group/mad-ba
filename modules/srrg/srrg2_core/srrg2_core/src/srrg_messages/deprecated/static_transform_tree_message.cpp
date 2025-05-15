#include "static_transform_tree_message.h"

namespace srrg2_core {

  StaticTransformTreeMessage::StaticTransformTreeMessage(const std::string& topic_,
                                                         const std::string& frame_id_,
                                                         const int& seq_,
                                                         const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY_NV(tree) {
  }

  BOSS_REGISTER_CLASS(StaticTransformTreeMessage);

} /* namespace srrg2_core */
