#pragma once
#include "packet_base.h"

namespace srrg2_core {

  //! @brief packets where the payload is an array of Eigen vector3f
  //!        this is almost fully deprecated.
  struct PacketArray : public PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    size_t num_points  = 0;
    size_t num_normals = 0;
    Vector3f* points   = 0;
    Vector3f* normals  = 0;

    void clear() {
      if (points)
        delete[] points;
      if (normals)
        delete[] normals;
      num_points  = 0;
      num_normals = 0;
      points      = 0;
      normals     = 0;
    }

    void allocate(size_t num_points_, size_t num_normals_) {
      num_points  = num_points_;
      num_normals = num_normals_;
      if (num_points) {
        points = new Vector3f[num_points];
      }
      if (num_normals) {
        normals = new Vector3f[num_normals];
      }
    }

    //! @brief ctor
    PacketArray(const uint8_t type_,
                const int size_          = 0,
                const Vector3f* points_  = 0,
                const Vector3f* normals_ = 0) :

      PacketBase(type_) {
      allocate(size_, normals_ ? size_ : 0);
      // tg memcpy makes gcc 9.3 sgotting
      if(points_){
        for (size_t i = 0; i < num_points; ++i) {
          points[i] = points_[i];
        }
      }

      if(normals_){
        for (size_t i = 0; i < num_normals; ++i) {
          normals[i] = normals_[i];
        }
      }
    }

    virtual ~PacketArray() {
      clear();
    }

    char* serialize(char* buffer) const override {
      buffer = putInBuffer(buffer, num_points);
      buffer = putInBuffer(buffer, num_normals);
      for (size_t i = 0; i < num_points; ++i) {
        const Vector3f& p = points[i];
        buffer            = putInBuffer(buffer, p);
      }
      for (size_t i = 0; i < num_normals; ++i) {
        const Vector3f& p = normals[i];
        buffer            = putInBuffer(buffer, p);
      }
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      clear();
      buffer = getFromBuffer(num_points, buffer);
      buffer = getFromBuffer(num_normals, buffer);
      allocate(num_points, num_normals);
      for (size_t i = 0; i < num_points; ++i) {
        Vector3f& p = points[i];
        buffer      = getFromBuffer(p, buffer);
      }
      for (size_t i = 0; i < num_normals; ++i) {
        Vector3f& p = normals[i];
        buffer      = getFromBuffer(p, buffer);
      }
      return buffer;
    }
  };

  template <uint8_t PacketType_>
  struct PacketArray_ : public PacketArray {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static constexpr uint8_t PacketType = PacketType_;
    using PayloadType                   = Vector3f;

    PacketArray_(const int size_          = 0,
                 const Vector3f* points_  = 0,
                 const Vector3f* normals_ = 0) :
      PacketArray(PacketType_, size_, points_, normals_) {
    }
  };

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief payload packets
  static constexpr uint8_t PACKET_TYPE_POINTS   = 0xE0;
  static constexpr uint8_t PACKET_TYPE_LINES    = 0xE1;
  static constexpr uint8_t PACKET_TYPE_SEGMENTS = 0xE2;

  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //
  //! @brief payload packet
  using PacketPayloadPoints   = PacketArray_<PACKET_TYPE_POINTS>;
  using PacketPayloadLines    = PacketArray_<PACKET_TYPE_LINES>;
  using PacketPayloadSegments = PacketArray_<PACKET_TYPE_SEGMENTS>;

} // namespace srrg2_core
