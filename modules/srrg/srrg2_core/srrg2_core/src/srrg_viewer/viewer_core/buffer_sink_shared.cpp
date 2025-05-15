#include "buffer_sink_shared.h"
#include "packet_deserializer.h"
#include "packet_serializer.h"

namespace srrg2_core {

  BufferSinkShared::~BufferSinkShared() {}

  void BufferSinkShared::putBuffer(BufferMemory* buffer_) {
    //ia in this case the buffer is shared between the two entities
    // if (not source_ptr)
    //   sgancell buffer
    // else
    //  mand buffer

    if (!param_manager_ptr.value())
      throw std::runtime_error("[BufferSinkShared::putBuffer] please set a valid BufferManager");

    if (!param_source_ptr.value()) {
      param_manager_ptr->freeBuffer(buffer_);
      return;
    }
    param_source_ptr->pushBuffer(buffer_);
  }
} //ia end namespace
