#include "message_sorted_sink.h"
#include "srrg_messages/messages/base_sensor_message.h"
#include <iomanip>

namespace srrg2_core {

  double MessageSortedSink::earliestStamp() {
    if (_msg_queue.size()) {
      return _msg_queue.begin()->first;
    }
    return -1.0;
  }

  double MessageSortedSink::latestStamp() {
    if (_msg_queue.size()) {
      return _msg_queue.rbegin()->first;
    }
    return -1.0;
  }

  bool MessageSortedSink::flush() {
    if (! _msg_queue.empty()) {
      BaseSensorMessagePtr msg = _msg_queue.begin()->second;
      _msg_queue.erase(_msg_queue.begin());
      return propagateMessage(msg);
    } else
      return MessageSinkBase::flush();
  }

  bool MessageSortedSink::isFlushed() {
    if (! _msg_queue.empty())
      return false;
    return MessageSinkBase::isFlushed();
  }

  void MessageSortedSink::resetCounters() {
    _num_dropped_messages = 0;
  }

  bool MessageSortedSink::putMessage(BaseSensorMessagePtr msg_) {
    if (!param_push_sinks.size()) {
      return false;
    }

    // if the message is too old, we drop it
    double crop_horizon = latestStamp() - param_oblivion_interval.value();
    if (msg_->timestamp.value() < crop_horizon) {
      ++_num_dropped_messages;
      if (param_verbose.value()) {
        std::cerr << "Dropping clnm:[" << msg_->className() << "] tpc:[" << msg_->topic.value() <<"]" << std::endl;
        std::cerr.precision(9);
        std::cerr << std::fixed;
        std::cerr <<  "ts: " << msg_->timestamp.value() << std::endl;
        std::cerr <<  "lt: " << latestStamp() << std::endl;
        std::cerr <<  "co: " << crop_horizon << std::endl;
        std::cerr <<  "co: " << msg_->timestamp.value() - crop_horizon << std::endl;
      }
      return false;
    }
    _msg_queue.insert(std::make_pair(msg_->timestamp.value(), msg_));
    bool success = false;
    while (latestStamp() - earliestStamp() > param_time_interval.value()) {
      BaseSensorMessagePtr msg = _msg_queue.begin()->second;
      _msg_queue.erase(_msg_queue.begin());
      success = propagateMessage(msg);
    }
    return success;
  }

  void MessageSortedSink::reset() {
    _msg_queue.clear();
    _num_dropped_messages = 0;
    MessageSinkBase::reset();
  }

} // namespace srrg2_core
