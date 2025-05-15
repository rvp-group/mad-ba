#include "buffer_source_shared.h"

namespace srrg2_core {

  BufferSourceShared::~BufferSourceShared() {}

  BufferMemory* BufferSourceShared::getBuffer() {
    if (!param_manager_ptr.value())
      throw std::runtime_error("[BufferSourceShared] please set a valid BufferManager");

    BufferMemory* b = param_manager_ptr->getBuffer(true); //ia get a buffer for reading purposes
    return b;
  }

  void BufferSourceShared::releaseBuffer(BufferMemory* buffer_) {
    if (!param_manager_ptr.value())
      throw std::runtime_error("[BufferSourceShared] please set a valid BufferManager");

    param_manager_ptr->releaseBuffer(buffer_);
  }

  BufferMemory* BufferSourceShared::getBufferTimeout() {
    if (!param_manager_ptr.value())
      throw std::runtime_error("[BufferSourceShared] please set a valid BufferManager");

    BufferMemory* b = param_manager_ptr->getBufferTimeout(true); //ia get a buffer for reading purposes
    return b;
  }

  void BufferSourceShared::pushBuffer(BufferMemory* buffer_) {
    if (!param_manager_ptr.value())
      throw std::runtime_error("[BufferSourceShared] please set a valid BufferManager");

    param_manager_ptr->releaseBuffer(buffer_);
  }

} //ia end namespace
