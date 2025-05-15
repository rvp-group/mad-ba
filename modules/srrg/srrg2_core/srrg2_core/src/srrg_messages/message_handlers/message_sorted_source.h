#pragma once
#include "message_filter_base.h"

namespace srrg2_core {

  class MessageSortedSource : public MessageFilterBase {
  public:
    PARAM(PropertyDouble, time_interval, "lag time to sort messages", 1., 0);

    MessageSortedSource();
    BaseSensorMessagePtr getMessage() override;

    int numDroppedMessages() const {
      return _num_dropped_messages;
    }

    void resetCounters();
    void reset() override;

  protected:
    double earliestStamp();
    inline double latestStamp();
    std::multimap<double, BaseSensorMessagePtr> _msg_queue;
    int _num_dropped_messages = 0;
  };

  using MessageSortedSourcePtr = std::shared_ptr<MessageSortedSource>;

} // namespace srrg2_core
