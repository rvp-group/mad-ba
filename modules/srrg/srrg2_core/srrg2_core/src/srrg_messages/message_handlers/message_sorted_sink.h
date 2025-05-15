#pragma once
#include "message_sink_base.h"

namespace srrg2_core {

  class MessageSortedSink : public MessageSinkBase {
  public:
    PARAM(PropertyDouble, time_interval, "lag time to sort messages", 1., 0);
    PARAM(PropertyDouble, oblivion_interval, "messages older than this lag that will be blasted, no matta what", 5., 0);
    PARAM(PropertyBool, verbose, "if set prints crap", false, 0);
    bool putMessage(BaseSensorMessagePtr msg_) override;

    int numDroppedMessages() const {
      return _num_dropped_messages;
    }

    void resetCounters();
    void reset() override;

    bool flush() override;

    bool isFlushed() override;

  protected:
    double earliestStamp();
    inline double latestStamp();
    std::multimap<double, BaseSensorMessagePtr> _msg_queue;
    int _num_dropped_messages = 0;
  };

  using MessageSortedSinkPtr = std::shared_ptr<MessageSortedSink>;
} // namespace srrg2_core
