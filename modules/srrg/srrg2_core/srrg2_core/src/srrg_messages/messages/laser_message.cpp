#include "laser_message.h"

namespace srrg2_core {

  LaserMessage::LaserMessage(const std::string& topic_,
                             const std::string& frame_id_,
                             const int& seq_,
                             const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(angle_min, 0),
    SETUP_PROPERTY(angle_max, 0),
    SETUP_PROPERTY(angle_increment, 0),
    SETUP_PROPERTY(time_increment, 0),
    SETUP_PROPERTY(scan_time, 0),
    SETUP_PROPERTY(range_min, 0),
    SETUP_PROPERTY(range_max, 0),
    SETUP_PROPERTY(ranges, std::vector<float>()),
    SETUP_PROPERTY(intensities, std::vector<float>()) {
  }

}  // namespace srrg2_core
