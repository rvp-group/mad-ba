#pragma once
#include "message_sink_base.h"
#include <set>

namespace srrg2_core {

  class MessageSelectorSink : public MessageSinkBase {
  public:
    PARAM_VECTOR(PropertyVector_<std::string>,
                 topics,
                 "topics to be propagated",
                 &_topics_changed_flag);

    MessageSelectorSink()          = default;
    virtual ~MessageSelectorSink() = default;

    bool putMessage(BaseSensorMessagePtr msg_) override;

  protected:
    bool _topics_changed_flag = true;

    void refreshMap();
    void putMessageRecursive(BaseSensorMessagePtr msg_);
    std::set<std::string> _topics;
  };

} // namespace srrg2_core
