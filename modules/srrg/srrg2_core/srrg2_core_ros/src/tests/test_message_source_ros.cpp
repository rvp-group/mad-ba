#include "srrg_messages_ros/message_handlers/message_ros_source.h"
#include "srrg_messages/message_handlers/message_pack.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

int main(int argc, char** argv) {

  const std::string imu_topic = "/imu";
  const std::string laser_topic = "/laser";
  const std::string range_topic = "/usound";
  const std::string tf_topic = "/tf";

  ros::init(argc, argv, "test_message_source_ros");
  MessageROSSource src;
  src.open();

  while (ros::ok()) {
    BaseSensorMessagePtr msg = src.getMessage();
    if (!msg) {
      continue;
    }
    MessagePackPtr pack = std::dynamic_pointer_cast<MessagePack>(msg);
    if (pack) {
      std::cerr << "got pack " << pack
                << " seq: " << pack->seq.value()
                << " with # " << pack->messages.size() << " messages" << std::endl;

      for (auto message_in_pack: pack->messages) {
        if (BaseSensorMessagePtr inner_msg = std::dynamic_pointer_cast<BaseSensorMessage>(message_in_pack)) {
          std::cerr << " - class name: " << inner_msg->className() << std::endl;
        }
      }
    }
  }
  return 0;
}
