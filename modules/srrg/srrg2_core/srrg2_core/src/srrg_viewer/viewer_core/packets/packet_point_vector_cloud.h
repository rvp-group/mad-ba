#pragma once
#include "packet_stl_vector.h"
#include "srrg_pcl/point_types.h"

namespace srrg2_core {

  //! @brief specialization of PacketVector_ that contains a point cloud
  template <uint8_t PacketType_, typename PointType_>
  struct PacketPointVector_ : public PacketVector_<PacketType_, PointType_> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    template <typename T_>
    friend class PacketCreator_;
    using BaseClass = PacketVector_<PacketType_, PointType_>;

    using PointType                     = PointType_;
    using EntryTypeVector               = typename BaseClass::EntryTypeVector;
    static constexpr uint8_t PacketType = PacketType_;

    //! @brief default ctor (calls base class)
    PacketPointVector_(const EntryTypeVector* point_vector_ = nullptr) : BaseClass(point_vector_) {
      // ia empty
    }

    //! @brief ctor that copies a subset of fields only
    // ds create an adapted packet vector of EntryType from a different
    // SourceEntryType_ ds this is useful to visualize only a subset of the
    // elements of SourceEntryType_ ds it is required that SourceEntryType_ is a
    // subclass of EntryType
    template <typename SourceEntryType_>
    PacketPointVector_(
      const std::vector<SourceEntryType_, Eigen::aligned_allocator<SourceEntryType_>>*
        point_vector_ = nullptr) {
      // ia initialize type
      this->type = PacketType_;

      if (point_vector_) {
        this->num_elements = point_vector_->size();

        // ds populate adapted vector element by element
        this->data_vector = new EntryTypeVector();
        this->data_vector->reserve(this->num_elements);
        for (const auto& point_source : *point_vector_) {
          // ds copy over all shared fields from the source into the destination
          // point
          PointType point;
          point.template copyFields<SourceEntryType_>(point_source);
          this->data_vector->emplace_back(point);
        }

        assert(this->data_vector->size() == point_vector_->size() &&
               "PacketPointVector_::PacketPointVector_|ERROR: bad construction");
      } else {
        BaseClass::clear();
      }
    }

    char* serialize(char* buffer) const override {
      return BaseClass::serialize(buffer);
    }

    const char* deserialize(const char* buffer) override {
      return BaseClass::deserialize(buffer);
    }
  };

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief vector pointcloud packets - float
  static constexpr uint8_t PACKET_TYPE_POINT_2F_VECTOR            = 0x30;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_2F_VECTOR     = 0x31;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_2F_VECTOR = 0x32;
  static constexpr uint8_t PACKET_TYPE_POINT_3F_VECTOR            = 0x33;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_3F_VECTOR     = 0x34;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_3F_VECTOR = 0x35;
  static constexpr uint8_t PACKET_TYPE_POINT_4F_VECTOR            = 0x36;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_4F_VECTOR     = 0x37;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_4F_VECTOR = 0x38;
  //! @brief vector pointcloud packets - double
  static constexpr uint8_t PACKET_TYPE_POINT_2D_VECTOR            = 0x39;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_2D_VECTOR     = 0x3A;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_2D_VECTOR = 0x3B;
  static constexpr uint8_t PACKET_TYPE_POINT_3D_VECTOR            = 0x3C;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_3D_VECTOR     = 0x3D;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_3D_VECTOR = 0x3E;
  static constexpr uint8_t PACKET_TYPE_POINT_4D_VECTOR            = 0x3F;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_4D_VECTOR     = 0x40;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_4D_VECTOR = 0x41;
  //! @brief vector pointcloud packets - int
  static constexpr uint8_t PACKET_TYPE_POINT_2I_VECTOR            = 0x42;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_2I_VECTOR     = 0x43;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_RGB_2I_VECTOR = 0x44;

  //! @point vector pointcloud packets - descriptors
  static constexpr uint8_t PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_2F_VECTOR = 0x45;
  static constexpr uint8_t PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_3F_VECTOR = 0x46;

  //! @breif polygonal meshes
  static constexpr uint8_t PACKET_TYPE_POLYGON_WIREFRAME_POINT_NORMAL_RGB_3F_VECTOR = 0x47;
  static constexpr uint8_t PACKET_TYPE_POLYGON_POINT_NORMAL_RGB_3F_VECTOR           = 0x48;

  //! @brief vector pointcloud packets - curvature
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_CURVATURE_3F_VECTOR = 0x49;
  static constexpr uint8_t PACKET_TYPE_POINT_NORMAL_CURVATURE_3D_VECTOR = 0x4A;

  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief vector pointclouds packets
  using PacketPoint2fVectorCloud = PacketPointVector_<PACKET_TYPE_POINT_2F_VECTOR, Point2f>;
  using PacketPointNormal2fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_2F_VECTOR, PointNormal2f>;
  using PacketPointNormalColor2fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_RGB_2F_VECTOR, PointNormalColor2f>;
  using PacketPoint3fVectorCloud = PacketPointVector_<PACKET_TYPE_POINT_3F_VECTOR, Point3f>;
  using PacketPointNormal3fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_3F_VECTOR, PointNormal3f>;
  using PacketPointNormalColor3fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_RGB_3F_VECTOR, PointNormalColor3f>;
  using PacketPoint4fVectorCloud = PacketPointVector_<PACKET_TYPE_POINT_4F_VECTOR, Point4f>;
  using PacketPointNormal4fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_4F_VECTOR, PointNormal4f>;
  using PacketPointNormalColor4fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_RGB_4F_VECTOR, PointNormalColor4f>;

  using PacketPoint2dVectorCloud = PacketPointVector_<PACKET_TYPE_POINT_2D_VECTOR, Point2d>;
  using PacketPointNormal2dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_2D_VECTOR, PointNormal2d>;
  using PacketPointNormalColor2dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_RGB_2D_VECTOR, PointNormalColor2d>;
  using PacketPoint3dVectorCloud = PacketPointVector_<PACKET_TYPE_POINT_3D_VECTOR, Point3d>;
  using PacketPointNormal3dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_3D_VECTOR, PointNormal3d>;

  using PacketPointNormalColor3dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_RGB_3D_VECTOR, PointNormalColor3d>;
  using PacketPoint4dVectorCloud = PacketPointVector_<PACKET_TYPE_POINT_4D_VECTOR, Point4d>;
  using PacketPointNormal4dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_4D_VECTOR, PointNormal4d>;
  using PacketPointNormalColor4dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_RGB_4D_VECTOR, PointNormalColor4d>;

  //! @brief point with descriptors
  using PacketPointIntensityDescriptor2fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_2F_VECTOR, Point2f>;
  using PacketPointIntensityDescriptor3fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_3F_VECTOR, Point3f>;

  //! @brief polygon as vector of points
  using PacketPolygonPointNormalColor3fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POLYGON_POINT_NORMAL_RGB_3F_VECTOR, PointNormalColor3f>;
  using PacketPolygonWireframePointNormalColor3fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POLYGON_WIREFRAME_POINT_NORMAL_RGB_3F_VECTOR,
                       PointNormalColor3f>;

  //! @brief point with curvature
  using PacketPointNormalCurvature3fVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_CURVATURE_3F_VECTOR, PointNormalCurvature3f>;
  using PacketPointNormalCurvature3dVectorCloud =
    PacketPointVector_<PACKET_TYPE_POINT_NORMAL_CURVATURE_3D_VECTOR, PointNormalCurvature3d>;

} // namespace srrg2_core
