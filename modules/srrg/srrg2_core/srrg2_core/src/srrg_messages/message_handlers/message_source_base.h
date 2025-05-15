#pragma once
#include "srrg_boss/deserializer.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "srrg_messages/messages/base_sensor_message.h"
#include "srrg_property/property_vector.h"

namespace srrg2_core {

  class MessageSourceBase : public Configurable {
  public:
    MessageSourceBase();
    virtual ~MessageSourceBase() = default;

    virtual MessageSourceBase* getRootSource() = 0;
    virtual BaseSensorMessagePtr getMessage()  = 0;

    inline virtual void open(){};
    inline bool isOpen() {
      return _is_open;
    }
    inline virtual void reset() {
    }
    volatile bool _running = true;

  protected:
    bool _is_open = false;
  };

  using MessageSourceBasePtr = std::shared_ptr<MessageSourceBase>;

} // namespace srrg2_core
