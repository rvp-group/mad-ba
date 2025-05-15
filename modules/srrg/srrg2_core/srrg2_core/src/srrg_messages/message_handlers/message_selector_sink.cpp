#include "message_selector_sink.h"
#include "message_pack.h"
namespace srrg2_core {

  void MessageSelectorSink::putMessageRecursive(BaseSensorMessagePtr msg_) {
    auto it = _topics.find(msg_->topic.value());
    if (it != _topics.end()) {
      propagateMessage(msg_);
      return;
    }
    MessagePackPtr pack = std::dynamic_pointer_cast<MessagePack>(msg_);
    if (!pack) {
      return;
    }
    for (size_t i = 0; i < pack->messages.size(); ++i) {
      putMessageRecursive(pack->messages[i]);
    }
  }

  bool MessageSelectorSink::putMessage(BaseSensorMessagePtr msg_) {
    if (_topics_changed_flag) {
      refreshMap();
      _topics_changed_flag = false;
    }
    putMessageRecursive(msg_);
    return true;
  }

  void MessageSelectorSink::refreshMap() {
    _topics.clear();
    for (size_t i = 0; i < param_topics.size(); ++i) {
      _topics.insert(param_topics.value(i));
    }
  }
  //  std::set<std::string> _topics;

} // namespace srrg2_core
