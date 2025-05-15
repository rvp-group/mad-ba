#include "buffer_memory.h"

namespace srrg2_core {

  BufferMemory::BufferMemory() {
    erase();
  }

  BufferMemory::~BufferMemory() {
    erase();
  }

  BufferMemory* BufferMemory::clone() const  {
    if (!size)
      return 0;
    BufferMemory* clone = new BufferMemory();
    clone->allocate(size);
    clone->num_packets = num_packets;
    clone->status = status;
    memcpy(clone->data, buffer_start, size);
    return clone;
  }

  void BufferMemory::allocate(const size_t& size_) {
    if (buffer_start)
      erase();
    data = new char[size_];
    buffer_start = data;

    if (!data)
      throw std::runtime_error("[Buffer] something went wrong in the allocation of the buffer");

    size = size_;
    num_packets = 0;
    status = BufferStatus::Free;
  }

  void BufferMemory::erase() {
    if (!buffer_start)
      return;
    delete [] buffer_start;
    data = 0;
    buffer_start = 0;
    size = 0;
    num_packets = 0;
    status = BufferStatus::Free;
  }


  void BufferMemory::clear() {
    if (!buffer_start)
      return;
    memset(buffer_start, 0, size);
    data = buffer_start;
    num_packets = 0;
    status = BufferStatus::Free;
  }

} //ia end namespace
