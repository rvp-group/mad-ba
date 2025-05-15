#pragma once
#include "message_source_base.h"
#include "srrg_config/property_configurable.h"
namespace srrg2_core {

  class MessageFilterBase : public MessageSourceBase {
  public:
    PARAM(PropertyConfigurable_<MessageSourceBase>,
          source,
          "the source from where this filter reads",
          nullptr,
          nullptr);

    virtual ~MessageFilterBase() = default;
    MessageSourceBase* getRootSource() override;
    BaseSensorMessagePtr getMessage() = 0;
    void reset() override;
  };

  using MessageFilterBasePtr = std::shared_ptr<MessageFilterBase>;

} // namespace srrg2_core
