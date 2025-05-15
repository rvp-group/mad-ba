#pragma once
#include "buffer_memory.h"
#include "packets/packet_base.h"

namespace srrg2_core {

  //! @brief serializes packets on an external buffer. it owns nothing.
  //!        you specify a packet and a type and it goes.
  class PacketSerializer {
  public:
    PacketSerializer();
    ~PacketSerializer();

    inline void setBuffer(BufferMemory* buffer_) {
      if (!buffer_->size)
        throw std::runtime_error("[PacketSerializer] setting an empty buffer");
      _buffer = buffer_;
    }

    void putPacket(const PacketBase& packet_);

  protected:
    BufferMemory* _buffer = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  };
} //ia end namespace
