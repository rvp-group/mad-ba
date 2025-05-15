#include "message_sorted_source.h"
#include "srrg_messages/messages/base_sensor_message.h"
#include "srrg_system_utils/shell_colors.h"

namespace srrg2_core {

  double MessageSortedSource::earliestStamp() {
    if (_msg_queue.size()) {
      return _msg_queue.begin()->first;
    }
    return -1.0;
  }

  double MessageSortedSource::latestStamp() {
    if (_msg_queue.size()) {
      return _msg_queue.rbegin()->first;
    }
    return -1.0;
  }

  void MessageSortedSource::resetCounters() {
    _num_dropped_messages = 0;
  }

  MessageSortedSource::MessageSortedSource() {
    std::cerr << RED << className() << "| is deprecated. Please revise your pipeline using sinks"
              << RESET << std::endl;
  }

  BaseSensorMessagePtr MessageSortedSource::getMessage() {
    assert(param_source.value() && "you need to set a valid source");
    MessageSourceBasePtr src = param_source.value();
    // we keep on reading messages until the
    // latest-earliest is below time_interval
    while (latestStamp() - earliestStamp() < param_time_interval.value()) {
      BaseSensorMessagePtr msg = src->getMessage();
      if (!msg) {
        break;
      }
      // if the message is too old we drop it
      if (msg->timestamp.value() < latestStamp() - param_time_interval.value()) {
        ++_num_dropped_messages;
        continue;
      }
      _msg_queue.insert(std::make_pair(msg->timestamp.value(), msg));
    }
    // we return the first message in the queue, if any
    if (_msg_queue.empty()) {
      std::cerr << __PRETTY_FUNCTION__ << ": source ova" << std::endl;
      return 0;
    }
    BaseSensorMessagePtr msg = _msg_queue.begin()->second;
    _msg_queue.erase(_msg_queue.begin());
    return msg;
  }

  void MessageSortedSource::reset() {
    _msg_queue.clear();
    _num_dropped_messages = 0;
    MessageFilterBase::reset();
  }

} // namespace srrg2_core
