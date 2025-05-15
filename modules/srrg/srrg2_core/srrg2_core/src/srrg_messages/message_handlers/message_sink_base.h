#pragma once
#include "srrg_boss/serializer.h"
#include "srrg_config/property_configurable_vector.h"
#include "srrg_data_structures/platform.h"
#include "srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core {

  class ViewerCanvas;

  class MessageSinkBase : public Configurable, public PlatformUser {
  public:
    PARAM(PropertyString, tf_topic, "name of the transform tree to subscribe to", "", nullptr);
    PARAM_VECTOR(PropertyConfigurableVector_<MessageSinkBase>,
                 push_sinks,
                 "sinks to which the output of this message will be pushed with propagateMessage",
                 nullptr);
    MessageSinkBase()          = default;
    virtual ~MessageSinkBase() = default;

    // default implementation just propagates the message
    virtual bool putMessage(BaseSensorMessagePtr msg_);

    // explicit method to propagate the message to the connected sinks
    virtual bool propagateMessage(BaseSensorMessagePtr msg_);

    // default implementation propagates the reset to all connected sinks
    void reset() override;

    void setPlatform(PlatformPtr) override;

    inline bool isOpen() {
      return _is_open;
    }

    //! @brief flushes internal buffer (if any)
    virtual bool flush();

    //! @brief checks if any internal buffer is unflushed
    virtual bool isFlushed();

  protected:
    bool _is_open = false;
  };

  using MessageSinkBasePtr = std::shared_ptr<MessageSinkBase>;
} // namespace srrg2_core
