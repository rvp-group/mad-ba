#include <srrg_messages/messages/point_stamped_message.h>

namespace srrg2_core {

  PointStampedMessage::PointStampedMessage(const std::string& topic_,
                             const std::string& frame_id_,
                             const int& seq_,
                             const double& timestamp_) :
      BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
      SETUP_PROPERTY(point, Vector3f(0, 0, 0)) {
  }

} /* namespace srrg2_slam_architecture */ // srrg2_slam_architecture
