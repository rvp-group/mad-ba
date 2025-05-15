#pragma once
#include "message_file_source_base.h"
#include "srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core {

  class MessageFileSource : public MessageFileSourceBase {
  public:
    virtual void open(const std::string& filename) override;
    virtual void open() override;
    virtual void close() override;

    MessageFileSource() = default;
    virtual ~MessageFileSource();
    BaseSensorMessagePtr getMessage() override;
    void reset() override;

  protected:
    std::unique_ptr<Deserializer> _deserializer = nullptr;
  };

  using MessageFileSourcePtr = std::shared_ptr<MessageFileSource>;

} // namespace srrg2_core
