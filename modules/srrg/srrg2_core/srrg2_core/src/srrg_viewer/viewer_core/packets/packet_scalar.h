#pragma once
#include "packet_base.h"

namespace srrg2_core {

  //! @brief packets where the payload is just a scalar
  //!        they can be used for different reasons (from objects to GL
  //!        commands)
  template <uint8_t PacketType_, typename Scalar_>
  struct PacketScalar_ : public PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    template <typename T_>
    friend class PacketCreator_;

    static constexpr uint8_t PacketType = PacketType_;
    using PayloadType                   = Scalar_;

    //! @brief the scalar that carries
    PayloadType data;

    PacketScalar_(const Scalar_& data_) : PacketBase(PacketType_) {
      data = data_;
    }

    virtual ~PacketScalar_() {
    }

    char* serialize(char* buffer) const override {
      buffer = putInBuffer(buffer, data);
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      buffer = getFromBuffer(data, buffer);
      return buffer;
    }

  protected:
    PacketScalar_() : PacketBase(PacketType_), data(0) {
      // ia empty
    }
  };

  //! @brief usings
  template <uint8_t PacketType_>
  using PacketChar_ = PacketScalar_<PacketType_, char>;
  template <uint8_t PacketType_>
  using PacketInt_ = PacketScalar_<PacketType_, int>;
  template <uint8_t PacketType_>
  using PacketFloat_ = PacketScalar_<PacketType_, float>;
  template <uint8_t PacketType_>
  using PacketDouble_ = PacketScalar_<PacketType_, double>;
  template <uint8_t PacketType_>
  using PacketUInt_ = PacketScalar_<PacketType_, size_t>;

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //

  static constexpr uint8_t PACKET_TYPE_POINT_SIZE      = 0x10;
  static constexpr uint8_t PACKET_TYPE_LINE_WIDTH      = 0x11;
  static constexpr uint8_t PACKET_TYPE_SPHERE          = 0x12;
  static constexpr uint8_t PACKET_TYPE_REFERENCE_FRAME = 0x13;
  static constexpr uint8_t PACKET_TYPE_DISK            = 0x14;
  
  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //

  using PacketAttributePointSize = PacketFloat_<PACKET_TYPE_POINT_SIZE>;
  using PacketAttributeLineWidth = PacketFloat_<PACKET_TYPE_LINE_WIDTH>;

  //@brief data -> float(radius)
  using PacketObjectSphere = PacketFloat_<PACKET_TYPE_SPHERE>;
  //@brief data -> float(line_width)
  using PacketObjectReferenceFrame = PacketFloat_<PACKET_TYPE_REFERENCE_FRAME>;
  //@brief data -> float(radius)
  using PacketObjectDisk = PacketFloat_<PACKET_TYPE_DISK>;
} // namespace srrg2_core
