#pragma once
#include "buffer_memory.h"
#include "packet_factory.h"

namespace srrg2_core {

  //! @brief serializes packets on an external buffer. it owns nothing.
  //!        you specify a packet and a type and it goes.
  class PacketDeserializer {
  public:
    PacketDeserializer();
    ~PacketDeserializer();

    inline void setBuffer(BufferMemory* buffer_) {
      if (!buffer_->size)
        throw std::runtime_error("PacketDeserializer::setBuffer|setting an empty buffer");
      _buffer = buffer_;
    }

    PacketBase* getPacket();

  protected:
    BufferMemory* _buffer = 0;
    PacketFactory* _factory = 0;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} //ia end namespace
