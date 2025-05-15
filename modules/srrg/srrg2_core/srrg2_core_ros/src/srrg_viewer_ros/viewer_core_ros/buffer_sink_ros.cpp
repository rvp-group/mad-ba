#include "buffer_sink_ros.h"

using namespace srrg2_core;

namespace srrg2_core_ros {

  BufferSinkRos::~BufferSinkRos() {}

  void BufferSinkRos::putBuffer(BufferMemory* buffer_) {
    if (!param_manager_ptr.value())
      throw std::runtime_error("BufferSinkRos::putBuffer|please set a valid BufferManager");

    if (param_publisher_ptr.value()) {
      //ia if there is a publisher connected, do something
      _is_active = param_publisher_ptr->publishBuffer(buffer_);
    } else {
      //ia else free buffer
      param_manager_ptr->freeBuffer(buffer_);
    }
  }

} //ia end namespace srrg2_core_ros
