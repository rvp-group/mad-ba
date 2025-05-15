#pragma once
#include "packet_base.h"
#include "srrg_pcl/point_types.h"

namespace srrg2_core {
  //! @brief specialization of packet that contains an organized point cloud (in
  //! matrix form)
  template <uint8_t PacketType_, typename PointType_>
  struct PacketPointMatrix_ : public PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    template <typename T_>
    friend class PacketCreator_;
    using PointType = PointType_;
    using EntryTypeMatrix =
      srrg2_core::Matrix_<PointType, Eigen::aligned_allocator<PointType>>;
    static constexpr uint8_t PacketType = PacketType_;

    size_t rows                  = 0;
    size_t cols                  = 0;
    EntryTypeMatrix* data_matrix = 0;

    void clear() {
      if (data_matrix) {
        delete data_matrix;
      }
      rows        = 0;
      cols        = 0;
      data_matrix = nullptr;
    }

    void allocate(const size_t& rows_, const size_t& cols_) {
      rows        = rows_;
      cols        = cols_;
      data_matrix = new EntryTypeMatrix(rows, cols);
      assert(data_matrix->cols() == cols && data_matrix->rows() == rows &&
             "PacketMatrix_::allocate|bad allocate");
    }

    // ia default argument for ctor is required for the factory
    PacketPointMatrix_(const EntryTypeMatrix* point_matrix_ = 0) :
      PacketBase(PacketType_) {
      if (point_matrix_) {
        rows        = point_matrix_->rows();
        cols        = point_matrix_->cols();
        data_matrix = new EntryTypeMatrix(*point_matrix_);
        assert(data_matrix->cols() == cols && data_matrix->rows() == rows &&
               "PacketMatrix_::PacketMatrix_|bad allocate");
      } else {
        clear();
      }
    }

    ~PacketPointMatrix_() {
      clear();
    }

    char* serialize(char* buffer) const override {
      assert(data_matrix && "PacketMatrix_::serialize|invalid data matrix");
      // ia first put the size
      buffer            = putInBuffer(buffer, rows);
      buffer            = putInBuffer(buffer, cols);
      const size_t size = data_matrix->size();
      // ia then put the meat - worst line of code ever (I really hate matrix
      // pointclouds)
      std::memcpy((void*) buffer,
                  (const void*) data_matrix->data().data(),
                  size * sizeof(PointType_));
      buffer += size * sizeof(PointType_);
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      clear();
      // ia first get the size
      buffer = getFromBuffer(rows, buffer);
      buffer = getFromBuffer(cols, buffer);

      // ia allocate and copy
      allocate(rows, cols); // ia this will also reindex the matrix
      const size_t size = data_matrix->size();

      std::memcpy((void*) data_matrix->data().data(),
                  (const void*) buffer,
                  size * sizeof(PointType_));
      buffer += size * sizeof(PointType_);
      return buffer;
    }
  };

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief matrix pointcloud packets - float
  static constexpr uint8_t PACKET_TYPE_POINT_2F_MATRIX            = 0x60;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_2F_MATRIX     = 0x61;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_2F_MATRIX = 0x62;
  static constexpr uint8_t PACKET_TYPE_POINT_3F_MATRIX            = 0x63;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_3F_MATRIX     = 0x64;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_3F_MATRIX = 0x65;
  static constexpr uint8_t PACKET_TYPE_POINT_4F_MATRIX            = 0x66;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_4F_MATRIX     = 0x67;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_4F_MATRIX = 0x68;
  //! @brief matrix pointcloud packets - double
  static constexpr uint8_t PACKET_TYPE_POINT_2D_MATRIX            = 0x69;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_2D_MATRIX     = 0x6A;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_2D_MATRIX = 0x6B;
  static constexpr uint8_t PACKET_TYPE_POINT_3D_MATRIX            = 0x6C;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_3D_MATRIX     = 0x6D;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_3D_MATRIX = 0x6E;
  static constexpr uint8_t PACKET_TYPE_POINT_4D_MATRIX            = 0x6F;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_4D_MATRIX     = 0x70;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_4D_MATRIX = 0x71;
  //! @brief matrix pointcloud packets - int
  static constexpr uint8_t PACKET_TYPE_POINT_2I_MATRIX            = 0x72;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_2I_MATRIX     = 0x73;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_2I_MATRIX = 0x74;

  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief matrix pointclouds packets
  using PacketPoint2fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_2F_MATRIX, Point2f>;
  using PacketPointNormal2fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_2F_MATRIX, PointNormal2f>;
  using PacketPointNormalColor2fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_RGB_2F_MATRIX,
                       PointNormalColor2f>;
  using PacketPoint3fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_3F_MATRIX, Point3f>;
  using PacketPointNormal3fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_3F_MATRIX, PointNormal3f>;
  using PacketPointNormalColor3fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_RGB_3F_MATRIX,
                       PointNormalColor3f>;
  using PacketPoint4fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_4F_MATRIX, Point4f>;
  using PacketPointNormal4fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_4F_MATRIX, PointNormal4f>;
  using PacketPointNormalColor4fMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_RGB_4F_MATRIX,
                       PointNormalColor4f>;

  using PacketPoint2dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_2D_MATRIX, Point2d>;
  using PacketPointNormal2dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_2D_MATRIX, PointNormal2d>;
  using PacketPointNormalColor2dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_RGB_2D_MATRIX,
                       PointNormalColor2d>;
  using PacketPoint3dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_3D_MATRIX, Point3d>;
  using PacketPointNormal3dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_3D_MATRIX, PointNormal3d>;
  using PacketPointNormalColor3dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_RGB_3D_MATRIX,
                       PointNormalColor3d>;
  using PacketPoint4dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_4D_MATRIX, Point4d>;
  using PacketPointNormal4dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_4D_MATRIX, PointNormal4d>;
  using PacketPointNormalColor4dMatrixCloud =
    PacketPointMatrix_<PACKET_TYPE_POINT_NORMAL_RGB_4D_MATRIX,
                       PointNormalColor4d>;
} // namespace srrg2_core
