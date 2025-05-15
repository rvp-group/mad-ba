#include "message_sink_base.h"

namespace srrg2_core {
  using namespace std;

  bool MessageSinkBase::putMessage(BaseSensorMessagePtr msg_) {
    return propagateMessage(msg_);
  }

  bool MessageSinkBase::propagateMessage(BaseSensorMessagePtr msg_) {
    bool success = false;
    for (size_t i = 0; i < param_push_sinks.size(); ++i) {
      MessageSinkBasePtr sink = param_push_sinks.value(i);
      if (!sink) {
        continue;
      }
      if (!sink->putMessage(msg_)) {
        success=true;
      }
    }
    return success;
  }

  void MessageSinkBase::reset() {
    //std::cerr << className() << "|reset" << std::endl;
    for (size_t i = 0; i < param_push_sinks.size(); ++i) {
      MessageSinkBasePtr sink = param_push_sinks.value(i);
      if (!sink) {
        continue;
      }
      sink->reset();
    }
  }

  void MessageSinkBase::setPlatform(PlatformPtr platform) {
    PlatformUser::setPlatform(platform);
    std::cerr << className() << "|setPlatform this: " << this << " _platform:" << platform.get() <<  std::endl;
  }

  //! @brief flushes internal buffer (if any)
  bool MessageSinkBase::flush() {
    if (isFlushed())
      return false;
    bool msg_processed=false;
    for (size_t i = 0; i < param_push_sinks.size(); ++i) {
      MessageSinkBasePtr sink = param_push_sinks.value(i);
      if (!sink) {
        continue;
      }
      msg_processed |= sink->flush();
    }
    return msg_processed;
  }

  //! @brief checks if any internal buffer is unflushed
  bool MessageSinkBase::isFlushed() {
    for (size_t i = 0; i < param_push_sinks.size(); ++i) {
      MessageSinkBasePtr sink = param_push_sinks.value(i);
      if (!sink) {
        continue;
      }
      if (!sink->isFlushed())
        return false;
    }
    return true;
  }

} // namespace srrg2_core
