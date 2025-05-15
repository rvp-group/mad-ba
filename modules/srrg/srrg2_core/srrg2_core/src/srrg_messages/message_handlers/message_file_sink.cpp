#include "message_file_sink.h"
#include "message_pack.h"

namespace srrg2_core {

  void MessageFileSink::open() {
    this->close();
    const std::string& fullname = param_filename.value();
    size_t lastindex            = fullname.find_last_of(".");
    if (lastindex == fullname.npos) {
      throw std::runtime_error("no valid path selected");
    }

    std::string rawname     = fullname.substr(0, lastindex);
    std::string binary_path = rawname + ".d/<classname>.<nameAttribute>.<id>.<ext>";
    _serializer.reset(new Serializer);
    _serializer->setFilePath(fullname);
    _serializer->setBinaryPath(binary_path);
    _is_open           = true;
    _file_changed_flag = false;
  }

  void MessageFileSink::open(const std::string& fullname) {
    MessageFileSinkBase::open(fullname);
  }

  void MessageFileSink::close() {
    if (!_is_open) {
      return;
    }
    _serializer.reset(nullptr);
    _is_open = false;
  }

  bool MessageFileSink::putMessage(BaseSensorMessagePtr msg_) {
    if (_file_changed_flag) {
      open();
    }
    if (!_serializer) {
      std::cerr << "invalid serialization context, did you call open() first" << std::endl;
      return false;
    }
    if (!msg_) {
      if (param_verbose.value()) {
        std::cerr << "null message" << std::endl;
      }
      return false;
    }
    // if it is a message pack, we serialize first all contained objects
    std::shared_ptr<MessagePack> pack = std::dynamic_pointer_cast<MessagePack>(msg_);
    if (pack) {
      for (size_t i = 0; i < pack->messages.size(); ++i) {
        BaseSensorMessagePtr m = pack->messages[i];
        if (!m) {
          std::cerr << "invalid message in pack" << std::endl;
          return false;
        }
        if (!putMessage(m)) {
          std::cerr << "error in serializing inner message in pack" << std::endl;
          return false;
        }
      }
    }
    return _serializer->writeObject(*msg_);
  }

  MessageFileSink::~MessageFileSink() {
    if (_is_open) {
      this->close();
    }
  }

} // namespace srrg2_core
