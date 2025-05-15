#pragma once
#include "message_handlers/message_ros_sink.h"
#include "message_handlers/message_ros_source.h"
#include "message_handlers/message_rosbag_sink.h"
#include "message_handlers/message_rosbag_source.h"
#include "srrg_messages/instances.h"

namespace srrg2_core_ros {
  void messages_ros_registerTypes() __attribute__((constructor));
}
