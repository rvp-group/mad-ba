#pragma once
#include "message_file_sink_base.h"

namespace srrg2_core {

  class MessageFileSink : public MessageFileSinkBase {
  public:
    ~MessageFileSink();
    virtual void open(const std::string& filename) override;
    virtual void open() override;
    virtual void close() override;
    bool putMessage(BaseSensorMessagePtr msg_) override;

  protected:
    std::unique_ptr<Serializer> _serializer = nullptr;
  };
  using MessageFileSinkPtr = std::shared_ptr<MessageFileSink>;

} // namespace srrg2_core
