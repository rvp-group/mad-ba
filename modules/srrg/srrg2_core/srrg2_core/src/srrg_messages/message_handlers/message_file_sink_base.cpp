#include "message_file_sink_base.h"

namespace srrg2_core {

  MessageFileSinkBase::~MessageFileSinkBase() {
    if (_is_open) {
      close();
    }
  }

  void MessageFileSinkBase::open(const std::string& filename_) {
    if (_is_open) {
      this->close();
    }
    param_filename.setValue(filename_);
    open();
  }

  void MessageFileSinkBase::close() {
    _is_open = false;
  }

} // namespace srrg2_core
