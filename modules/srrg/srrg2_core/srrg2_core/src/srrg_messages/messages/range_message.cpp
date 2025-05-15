#include "range_message.h"

namespace srrg2_core {

  RangeMessage::RangeMessage(const std::string& topic_,
                             const std::string& frame_id_,
                             const int& seq_,
                             const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(range_min, 0),
    SETUP_PROPERTY(range_max, 10),
    SETUP_PROPERTY(field_of_view, M_PI),
    SETUP_PROPERTY(radiation_type, RadiationType::ULTRASOUND),
    SETUP_PROPERTY(range, 0) {
  }

} // namespace srrg2_core
