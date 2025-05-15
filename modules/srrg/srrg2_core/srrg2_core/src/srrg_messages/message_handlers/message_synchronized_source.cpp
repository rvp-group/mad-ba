#include "message_synchronized_source.h"
#include "message_pack.h"
#include "srrg_system_utils/shell_colors.h"

namespace srrg2_core {

  void MessageSynchronizedSource::clearBuffer() {
    // we delete all messages in the buffers
    for (auto it = _message_map.begin(); it != _message_map.end(); ++it) {
      if (it->second) {
        it->second.reset();
      }
    }
  }

  void MessageSynchronizedSource::handleTopicsChanged() {
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

  void MessageSynchronizedSource::handleIntervalChanged() {
    resetCounters();
    _interval_changed = false;
  }

  MessageSynchronizedSource::MessageSynchronizedSource() {
    std::cerr << RED << className() << "| is deprecated. Please revise your pipeline using sinks"
              << RESET << std::endl;
  }

  BaseSensorMessagePtr MessageSynchronizedSource::getMessage() {
    //    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    assert(param_source.value() && "you need to set a valid source");
    MessageSourceBasePtr src = param_source.value();

    if (_topics_changed) {
      //      std::cerr << "topics changed" << std::endl;
      handleTopicsChanged();
    }
    if (_interval_changed) {
      //      std::cerr << "interval changed" << std::endl;
      handleIntervalChanged();
    }
    while (!isPacketReady()) {
      BaseSensorMessagePtr msg = src->getMessage();
      if (!msg) {
        //        std::cerr << __PRETTY_FUNCTION__ << ": source ova" << std::endl;
        return nullptr;
      }
      //      std::cerr << "got msg " << msg->topic.value() << std::endl;

      // find a bucket;
      auto it = _message_map.find(msg->topic.value());
      if (it == _message_map.end()) {
        //        std::cerr << "msg not map, deleting " << std::endl;
        // delete msg;
        msg.reset();
        continue;
      }

      // we assume the message is more recent than the previous
      // one and we replace it
      if (it->second) {
        //        std::cerr << "replacing old msg " << it->second->timestamp.value() << std::endl;
        ++_num_dropped_messages;
        // delete it->second;
        it->second.reset();
      }
      it->second = msg;
      //      std::cerr << "new ts: " << it->second->timestamp.value() << std::endl;
    }
    // we come here if all messages are synchronized
    // we need to assemble them in a packet
    // in the order of the topics
    MessagePack* pack =
      new MessagePack(param_output_topic.value(), param_output_frame_id.value(), _seq, _t_min);
    for (size_t i = 0; i < param_topics.size(); ++i) {
      const std::string& t = param_topics.value(i);
      auto it              = _message_map.find(t);
      assert(it != _message_map.end() && "no messge in map");
      assert(it->second && "invalid pointer");
      pack->messages.push_back(BaseSensorMessagePtr(it->second));
      it->second = 0;
    }
    ++_seq;
    return BaseSensorMessagePtr(pack);
  }

  bool MessageSynchronizedSource::isPacketReady() {
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

  void MessageSynchronizedSource::resetCounters() {
    _num_dropped_messages = 0;
  }

  void MessageSynchronizedSource::reset() {
    handleTopicsChanged();
    _seq = 0;
    MessageFilterBase::reset();
  }
} // namespace srrg2_core
