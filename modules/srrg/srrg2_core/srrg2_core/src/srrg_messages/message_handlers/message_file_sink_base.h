#pragma once
#include "message_sink_base.h"

namespace srrg2_core {

  class MessageFileSinkBase : public MessageSinkBase {
  public:
    PARAM(PropertyBool, verbose, "verbose", false, 0);
    PARAM(PropertyString, filename, "file to write", "", &_file_changed_flag);

  public:
    MessageFileSinkBase() {
      this->_is_open = false;
    }

    virtual ~MessageFileSinkBase();
    virtual void open(const std::string& filename);
    virtual void open() = 0;
    virtual void close();

  protected:
    bool _file_changed_flag = true;
  };

  using MessageFileSinkBasePtr = std::shared_ptr<MessageFileSinkBase>;

} // namespace srrg2_core
