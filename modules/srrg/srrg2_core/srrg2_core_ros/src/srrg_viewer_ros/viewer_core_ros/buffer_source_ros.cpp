#include "buffer_source_ros.h"

namespace srrg2_core_ros {

  using namespace srrg2_core;


  BufferSourceRos::~BufferSourceRos() {}


  BufferMemory* BufferSourceRos::getBuffer() {
    if (!param_manager_ptr.value())
      throw std::runtime_error("BufferSourceRos::getBuffer|please set a valid BufferManager");

    BufferMemory* b = param_manager_ptr->getBuffer(true); //ia get a buffer for reading purposes
    return b;
  }


  void BufferSourceRos::releaseBuffer(srrg2_core::BufferMemory* buffer_) {
    if (!param_manager_ptr.value())
      throw std::runtime_error("BufferSourceRos::getBuffer|please set a valid BufferManager");

    param_manager_ptr->releaseBuffer(buffer_); //ia release a buffer
  }

} //ia end  namespace srrg2_core_ros
