#include "instances.h"
namespace srrg2_core_ros {

  void messages_ros_registerTypes() {
    srrg2_core::messages_registerTypes();
    BOSS_REGISTER_CLASS(MessageROSSink);
    BOSS_REGISTER_CLASS(MessageROSSource);
    BOSS_REGISTER_CLASS(MessageROSBagSink);
    BOSS_REGISTER_CLASS(MessageROSBagSource);
  }
}
