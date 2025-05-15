#pragma once
#include "packet_base.h"

namespace srrg2_core {
  //! @breif packets whose payload is a single Eigen object
  //!        (Vector, Matrix, Isometry ...). It can be used for different things
  //!        (objects, commands ...)
  template <uint8_t PacketType_, typename EigenType_>
  struct PacketEigen_ : public PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    template <typename T_>
    friend class PacketCreator_;

    static constexpr uint8_t PacketType = PacketType_;
    using PayloadType                   = EigenType_;

    //! @brief the eigen payload
    PayloadType data;

    PacketEigen_(const EigenType_& data_) : PacketBase(PacketType_) {
      data = data_;
    }

    virtual ~PacketEigen_() {
      // ia empty
    }

    char* serialize(char* buffer) const override {
      std::memcpy(
        (void*) buffer, (const void*) data.data(), sizeof(EigenType_));
      buffer += sizeof(EigenType_);

      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      std::memcpy(
        (void*) data.data(), (const void*) buffer, sizeof(EigenType_));
      buffer += sizeof(EigenType_);
      return buffer;
    }

  protected:
    // ia needed for the goddamn factory
    PacketEigen_() : PacketBase(PacketType_) {
      data.setZero();
    }
  };

  //! @brief usings
  template <uint8_t PacketType_>
  using PacketVector2f_ = PacketEigen_<PacketType_, Vector2f>;
  template <uint8_t PacketType_>
  using PacketVector3f_ = PacketEigen_<PacketType_, Vector3f>;
  template <uint8_t PacketType_>
  using PacketVector4f_ = PacketEigen_<PacketType_, Vector4f>;

  template <uint8_t PacketType_>
  using PacketMatrix2f_ = PacketEigen_<PacketType_, Matrix2f>;
  template <uint8_t PacketType_>
  using PacketMatrix3f_ = PacketEigen_<PacketType_, Matrix3f>;
  template <uint8_t PacketType_>
  using PacketMatrix4f_ = PacketEigen_<PacketType_, Matrix4f>;

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief attribute packets
  static constexpr uint8_t PACKET_TYPE_COLOR_RGB  = 0x20;
  static constexpr uint8_t PACKET_TYPE_COLOR_RGBA = 0x21;

  //! @brief object packets - i.e. sphere, pyramid ...
  static constexpr uint8_t PACKET_TYPE_PLANE             = 0x22;
  static constexpr uint8_t PACKET_TYPE_BOX               = 0x23;
  static constexpr uint8_t PACKET_TYPE_BOX_WIREFRAME     = 0x24;
  static constexpr uint8_t PACKET_TYPE_PYRAMID           = 0x25;
  static constexpr uint8_t PACKET_TYPE_PYRAMID_WIREFRAME = 0x26;
  static constexpr uint8_t PACKET_TYPE_ELLIPSOID         = 0x27;
  static constexpr uint8_t PACKET_TYPE_CONE              = 0x28;
  static constexpr uint8_t PACKET_TYPE_CYLINDER          = 0x29;
  static constexpr uint8_t PACKET_TYPE_ARROW2D           = 0x2A;

  //! @brief transforms
  static constexpr uint8_t PACKET_TYPE_MULT_TRASLATION = 0x2B;
  static constexpr uint8_t PACKET_TYPE_MULT_SCALE      = 0x2C;
  static constexpr uint8_t PACKET_TYPE_MULT_ROTATION   = 0x2D;
  static constexpr uint8_t PACKET_TYPE_MULT_MATRIX4F   = 0x2E;

  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //

  using PacketAttributeColorRGBA = PacketVector4f_<PACKET_TYPE_COLOR_RGBA>;
  using PacketAttributeColorRGB  = PacketVector3f_<PACKET_TYPE_COLOR_RGB>;

  //! @brief object packet
  //@brief data -> Vector2f(lenght, width)
  using PacketObjectPlane = PacketVector2f_<PACKET_TYPE_PLANE>;
  //@brief data -> Vector3f(lenght, width,height)
  using PacketObjectBox = PacketVector3f_<PACKET_TYPE_BOX>;
  //@brief data -> Vector3f(lenght,width, height)
  using PacketObjectBoxWireframe = PacketVector3f_<PACKET_TYPE_BOX_WIREFRAME>;
  //@brief data -> Vector2f(lenght,height)
  using PacketObjectPyramid = PacketVector2f_<PACKET_TYPE_PYRAMID>;
  //@brief data -> Vector2f(lenght, height)
  using PacketObjectPyramidWireframe =
    PacketVector2f_<PACKET_TYPE_PYRAMID_WIREFRAME>;
  //@brief data -> Vector3f(rx, ry, rz)
  using PacketObjectEllipsoid = PacketVector3f_<PACKET_TYPE_ELLIPSOID>;
  //@brief data -> Vector2f(radius, height)
  using PacketObjectCone = PacketVector2f_<PACKET_TYPE_CONE>;
  //@brief data -> Vector2f(radius,height)
  using PacketObjectCylinder = PacketVector2f_<PACKET_TYPE_CYLINDER>;
  //@brief data -> Vector3f(len,head_width, head_len)
  using PacketObjectArrow2D = PacketVector3f_<PACKET_TYPE_ARROW2D>;

  //! @brief transformations
  using PacketTransformMultTraslation =
    PacketVector3f_<PACKET_TYPE_MULT_TRASLATION>;
  using PacketTransformMultScale = PacketVector3f_<PACKET_TYPE_MULT_SCALE>;
  using PacketTransformMultRotation =
    PacketMatrix3f_<PACKET_TYPE_MULT_ROTATION>;
  using PacketTransformMultMatrix = PacketMatrix4f_<PACKET_TYPE_MULT_MATRIX4F>;

} // namespace srrg2_core
