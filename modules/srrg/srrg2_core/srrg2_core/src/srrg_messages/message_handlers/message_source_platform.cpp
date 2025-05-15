#include "message_source_platform.h"
#include "srrg_system_utils/shell_colors.h"

namespace srrg2_core {

  MessageSourcePlatform::MessageSourcePlatform() {
    std::cerr << RED << className() << "| is deprecated. Please revise your pipeline using sinks"
              << RESET << std::endl;
  }

  void MessageSourcePlatform::bindTfTopics() {
    _tf_topics_binded = false;
    for (const std::string& t : param_tf_topics.value()) {
      StringPlatformPtrMap::iterator p_it = _platforms_map.find(t);
      if (p_it == _platforms_map.end()) {
        std::cerr << "MessageSourcePlatform::bindTfTopics|binding TF topic " << FG_YELLOW(t)
                  << " to a transform tree\n";
        _platforms_map.insert(std::make_pair(t, PlatformPtr(new Platform)));
      }
    }
    _tf_topics_binded = true;
    return;
  }

  BaseSensorMessagePtr MessageSourcePlatform::getMessage() {
    assert(param_source.value() &&
           "MessageSourcePlatform::getMessage|you need to set a valid source");

    if (!_tf_topics_binded) {
      bindTfTopics();
    }

    BaseSensorMessagePtr msg = param_source->getMessage();
    // ia source ended
    if (!msg) {
      std::cerr << "MessageSourcePlatform::getMessage|source ended" << std::endl;
      return nullptr;
    }

    StringPlatformPtrMap::const_iterator p_it = _platforms_map.find(msg->topic.value());
    if (p_it != _platforms_map.end() && (!p_it->second->add(msg))) {
      std::cerr << FG_RED("problems with topic: ");
      std::cerr << FG_BLUE(msg->topic.value()) << std::endl;
      throw std::runtime_error("MessageSourcePlatform::getMessage|invalid TF topic");
    }
    return msg;
  }

} /* namespace srrg2_core */
