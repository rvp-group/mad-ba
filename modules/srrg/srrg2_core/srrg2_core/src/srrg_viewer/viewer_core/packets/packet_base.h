#pragma once
#include "srrg_viewer/viewer_core/color_palette.h"

namespace srrg2_core {

  //! @brief auxiliary functions for serialization
  template <typename T>
  char* putInBuffer(char* buffer, const T& value) {
    T* b = (T*) buffer;
    *b   = value;
    return buffer + sizeof(T);
  }

  template <typename T>
  const char* getFromBuffer(T& value, const char* buffer) {
    const T* b = (const T*) buffer;
    //! TODO why I have to use the constructor again?? otherwise it segfaults
    //! with Eigen stuff
    value = T(*b);
    return buffer + sizeof(T);
  }

  //! @brief base packet. all the others derive from this.
  //!        it is characterized by its type, a UNIQUE uint8_t.
  struct PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    uint8_t type;
    PacketBase(int type_) {
      type = type_;
    }
    virtual ~PacketBase(){};
    virtual char* serialize(char* buffer) const         = 0;
    virtual const char* deserialize(const char* buffer) = 0;
  };

  //! @brief packet that specifies a command (like push, pop).
  //!        it has no payload.
  struct PacketCommand : PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PacketCommand(const uint8_t type_) : PacketBase(type_) {
    }

    virtual ~PacketCommand() {
    }

    char* serialize(char* buffer) const override {
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      return buffer;
    }
  };

  template <uint8_t PacketType_>
  struct PacketCommand_ : public PacketCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static constexpr uint8_t PacketType = PacketType_;
    PacketCommand_() : PacketCommand(PacketType_) {
    }
  };

  //! @brief first specification of packet. it contains only
  //!        a sequence number
  struct PacketInfo : PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using PayloadType = uint64_t;

    uint64_t sequence;
    PacketInfo(const uint8_t type_, const uint64_t& sequence_ = 0) :
      PacketBase(type_) {
      sequence = sequence_;
    }

    virtual ~PacketInfo() {
    }

    char* serialize(char* buffer) const override {
      buffer = putInBuffer(buffer, sequence);
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      buffer = getFromBuffer(sequence, buffer);
      return buffer;
    }
  };

  template <uint8_t PacketType_>
  struct PacketInfo_ : PacketInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static constexpr uint8_t PacketType = PacketType_;
    using PayloadType                   = uint64_t;
    PacketInfo_(const uint64_t& sequence_ = 0) :
      PacketInfo(PacketType_, sequence_) {
    }
  };

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief invalid packet
  static constexpr uint8_t PACKET_TYPE_INVALID = 0x00;
  //! @brief end epoch packet
  static constexpr uint8_t PACKET_TYPE_END_EPOCH = 0xFF;
  //! @brief command packets
  static constexpr uint8_t PACKET_TYPE_PUSH_MATRIX     = 0x01;
  static constexpr uint8_t PACKET_TYPE_POP_MATRIX      = 0x02;
  static constexpr uint8_t PACKET_TYPE_PUSH_COLOR      = 0x03;
  static constexpr uint8_t PACKET_TYPE_PUSH_POINT_SIZE = 0x04;
  static constexpr uint8_t PACKET_TYPE_PUSH_LINE_WIDTH = 0x05;
  static constexpr uint8_t PACKET_TYPE_POP_ATTRIBUTE   = 0x06;

  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief info packet
  using PacketInfoEndEpoch = PacketInfo_<PACKET_TYPE_END_EPOCH>;

  //! @brief command packet
  using PacketCommandPushMatrix = PacketCommand_<PACKET_TYPE_PUSH_MATRIX>;
  using PacketCommandPopMatrix  = PacketCommand_<PACKET_TYPE_POP_MATRIX>;
  using PacketCommandPushColor  = PacketCommand_<PACKET_TYPE_PUSH_COLOR>;
  using PacketCommandPushPointSize =
    PacketCommand_<PACKET_TYPE_PUSH_POINT_SIZE>;
  using PacketCommandPushLineWidth =
    PacketCommand_<PACKET_TYPE_PUSH_LINE_WIDTH>;
  using PacketCommandPopAttribute = PacketCommand_<PACKET_TYPE_POP_ATTRIBUTE>;

} // namespace srrg2_core
