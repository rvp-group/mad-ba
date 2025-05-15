#pragma once
#include "../messages/base_sensor_message.h"
#include "message_sink_base.h"
#include "srrg_boss/serializer.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "srrg_config/property_configurable_vector.h"
#include "srrg_data_structures/platform.h"

namespace srrg2_core {

  class MessagePlatformListenerSink : public MessageSinkBase {
  public:
    MessagePlatformListenerSink()          = default;
    virtual ~MessagePlatformListenerSink() = default;

    // default implementation just propagates the message
    virtual bool putMessage(BaseSensorMessagePtr msg_);

    // default implementation propagates the reset to all connected sinks
    void reset() override;

    //! @brief binds each TF topic to a transform tree,
    //!        allocates data structures and so on.
    void bindTfTopics();

    //! @brief get a platform from its TF topic name
    //!        returns 0 if not found.
    inline PlatformPtr platform(const std::string& tf_name_) const {
      auto it = _platforms_map.find(tf_name_);
      return (it != _platforms_map.end()) ? it->second : nullptr;
    }

  protected:
    StringPlatformPtrMap _platforms_map;
    bool _tf_topics_binded = false;
    void bindTfTopics(MessageSinkBasePtr sink_, std::set<MessageSinkBase*>& visited_);
  };

  using MessagePlatformListenerSinkPtr = std::shared_ptr<MessagePlatformListenerSink>;
} // namespace srrg2_core
