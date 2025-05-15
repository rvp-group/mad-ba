#include "message_platform_listener_sink.h"
#include <typeinfo>
#include <unistd.h>

namespace srrg2_core {

  void MessagePlatformListenerSink::reset() {
    _platform.reset();
    _platforms_map.clear();
    _tf_topics_binded = false;
    MessageSinkBase::reset();
  }

  bool MessagePlatformListenerSink::putMessage(BaseSensorMessagePtr msg) {
    if (!msg) {
      return false;
    }
    if (!_tf_topics_binded) {
      // throw std::runtime_error("MessagePlatformListenerSink::getMessage|did you forget to call
      // the bindTfTopics function?");
      bindTfTopics();
    }

    const std::string& msg_topic = msg->topic.value();

    auto p_it = _platforms_map.find(msg_topic);
    // srrg when I found a match in the tf_tree
    if (p_it != _platforms_map.end()) {
      PlatformPtr platform = p_it->second;
      // srrg I add the message to the platform
      bool res = p_it->second->add(msg);
      // srrg if something went wrong throw an error
      if (!res) {
        std::cerr << FG_RED("problems with topic: ");
        std::cerr << FG_BLUE(msg_topic) << std::endl;
        throw std::runtime_error("MessagePlatformListenerSink::getMessage|invalid TF topic");
      }
    }
    bool processed = propagateMessage(msg);
    return processed;
  }

  void MessagePlatformListenerSink::bindTfTopics() {
    std::cerr << "binding topics" << std::endl;
    std::set<MessageSinkBase*> visited;
    for (size_t i = 0; i < param_push_sinks.size(); ++i) {
      bindTfTopics(param_push_sinks.value(i), visited);
    }
    for (auto it : _platforms_map) {
      std::cerr << "tf_topic: " << it.first << std::endl;
    }
    _tf_topics_binded = true;
    std::cerr << "topics binded" << std::endl;
  }

  void MessagePlatformListenerSink::bindTfTopics(MessageSinkBasePtr sink_,
                                                 std::set<MessageSinkBase*>& visited_) {
    if (!sink_) {
      return;
    }
    if (visited_.find(sink_.get()) != visited_.end()) {
      return;
    }

    std::string tf_topic  = sink_->param_tf_topic.value();
    PlatformPtr platform_ = platform(tf_topic);
    if (tf_topic != "") {
      visited_.insert(sink_.get());
      if (!platform_) {
        platform_ = PlatformPtr(new Platform);
        _platforms_map.insert(std::make_pair(tf_topic, platform_));
        std::cerr << "new platform, topic:" << tf_topic << " ptr: " << platform_.get() << std::endl;
      }
      sink_->setPlatform(platform_);
    }

    // srrg try to push the platform to the following sinks
    for (size_t i = 0; i < sink_->param_push_sinks.size(); ++i) {
      bindTfTopics(sink_->param_push_sinks.value(i), visited_);
    }
  }

} /* namespace srrg2_core */
