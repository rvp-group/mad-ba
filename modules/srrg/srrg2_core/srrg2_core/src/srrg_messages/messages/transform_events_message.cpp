#include "transform_events_message.h"

namespace srrg2_core {

  TransformEventsMessage::TransformEventsMessage(const std::string& topic_,
                                                 const std::string& frame_id_,
                                                 const int& seq_,
                                                 const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY_NV(events) {
  }

  using TransformEventsMessagePtr = std::shared_ptr<TransformEventsMessage>;
} // namespace srrg2_core
