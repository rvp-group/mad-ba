#include "message_synchronized_sink.h"
#include "message_pack.h"

namespace srrg2_core {

  void MessageSynchronizedSink::clearBuffer() {
    // we delete all messages in the buffers
    for (auto it = _message_map.begin(); it != _message_map.end(); ++it) {
      if (it->second) {
        it->second.reset();
      }
    }
  }

  void MessageSynchronizedSink::handleTopicsChanged() {
    resetCounters();
    clearBuffer();
    _message_map.clear();
    for (size_t i = 0; i < param_topics.size(); ++i) {
      const std::string& t = param_topics.value(i);
      _message_map.insert(std::make_pair(t, nullptr));
    }
    _topics_changed = false;
    _seq            = 0;
  }

  void MessageSynchronizedSink::handleIntervalChanged() {
    resetCounters();
    _interval_changed = false;
  }

  bool MessageSynchronizedSink::putMessage(BaseSensorMessagePtr msg_) {
    if (_topics_changed) {
      //      std::cerr << "topics changed" << std::endl;
      handleTopicsChanged();
    }
    if (_interval_changed) {
      //      std::cerr << "interval changed" << std::endl;
      handleIntervalChanged();
    }

    auto it = _message_map.find(msg_->topic.value());
    if (it == _message_map.end()) {
      return true;
    }

    // we assume the message is more recent than the previous
    // one and we replace it
    ++_num_dropped_messages;
    it->second = msg_;

    if (!isPacketReady()) {
      return false;
    }

    // std::cerr << "new ts: " << it->second->timestamp.value() << std::endl;
    // we come here if all messages are synchronized
    // we need to assemble them in a packet
    // in the order of the topics
    MessagePackPtr pack(
      new MessagePack(param_output_topic.value(), param_output_frame_id.value(), _seq, _t_min));
    for (size_t i = 0; i < param_topics.size(); ++i) {
      const std::string& t = param_topics.value(i);
      auto it              = _message_map.find(t);
      assert(it != _message_map.end() && "no messge in map");
      assert(it->second && "invalid pointer");
      pack->messages.push_back(BaseSensorMessagePtr(it->second));
      it->second = 0;
    }
    ++_seq;
    return propagateMessage(pack);
  }

  bool MessageSynchronizedSink::isPacketReady() {
    _t_min = std::numeric_limits<double>::max();
    _t_max = 0;
    // a pack is ready if the lag between t_max and t_min is below time_interval
    for (auto it = _message_map.begin(); it != _message_map.end(); ++it) {
      if (!it->second) {
        return false;
      }
      _t_min = std::min(_t_min, it->second->timestamp.value());
      _t_max = std::max(_t_max, it->second->timestamp.value());
    }
    return (_t_max - _t_min) < param_time_interval.value();
  }

  void MessageSynchronizedSink::resetCounters() {
    _num_dropped_messages = 0;
  }

  void MessageSynchronizedSink::reset() {
    handleTopicsChanged();
    _seq = 0;
    MessageSinkBase::reset();
  }
} // namespace srrg2_core
