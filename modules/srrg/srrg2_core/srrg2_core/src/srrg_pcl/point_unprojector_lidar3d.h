#pragma once
#include "srrg_pcl/lidar_3d_utils/lidar_3d_lookup_projector.h"
#include "srrg_pcl/point_unprojector.h"

namespace srrg2_core {

  template <srrg2_lidar3d_utils::LIDAR_TYPE lidar3d_type_, typename DestPointCloudType_>
  class PointUnprojectorLidar3D_
    : public srrg2_core::PointUnprojectorBase_<DestPointCloudType_>,
      public srrg2_lidar3d_utils::Lidar3DLookupProjector_<lidar3d_type_> {
  public:
    // ia a lot of usings
    using ThisType               = PointUnprojectorLidar3D_<lidar3d_type_, DestPointCloudType_>;
    using BaseType               = srrg2_core::PointUnprojectorBase_<DestPointCloudType_>;
    using LidarProjectorBaseType = srrg2_lidar3d_utils::Lidar3DLookupProjector_<lidar3d_type_>;
    using DestPointCloudType     = DestPointCloudType_;
    using DestPointType          = typename DestPointCloudType::PointType;
    using DestMatrixType         = srrg2_core::PointCloud_<
      srrg2_core::Matrix_<DestPointType, Eigen::aligned_allocator<DestPointType>>>;
    using IsometryType = srrg2_core::Isometry_<float, DestPointType::GeometryDim>;
    using CameraMatrixType =
      Eigen::Matrix<float, DestPointType::GeometryDim, DestPointType::GeometryDim>;
    using Lidar3DSensorType           = srrg2_lidar3d_utils::Lidar3DSensor_<lidar3d_type_>;
    static constexpr int GeometryDim  = DestPointType::GeometryDim;
    static constexpr int ProjectedDim = DestPointType::GeometryDim - 1;
    static constexpr srrg2_lidar3d_utils::LIDAR_TYPE LidarType = lidar3d_type_;
    using ImageCoordinatesType = srrg2_core::Vector_<float, ProjectedDim>;

    PARAM(srrg2_core::PropertyUnsignedInt,
          num_columns,
          "number of columns for the projected image",
          870,
          &_initialization_required);

    PARAM(srrg2_core::PropertyFloat,
          horizontal_start_angle,
          "horizontal starting angle for the spherical projection [rad]",
          M_PI,
          &_initialization_required);
    PARAM(srrg2_core::PropertyFloat,
          horizontal_end_angle,
          "horizontal ending angle for the spherical projection [rad]",
          -M_PI,
          &_initialization_required);

    //! @brief compute the organized unprojection starting from an un-organized cloud and reserving
    //! a slot for normals
    template <srrg2_core::PointUnprojectorMode payload_offset_idx = srrg2_core::NoNormals,
              typename SrcChannelType,
              typename... SrcChannelsRestType>
    int computeMatrix(DestMatrixType& dest,
                      const SrcChannelType& src,
                      const SrcChannelsRestType&... rest);

    //! @brief compute the un-organized unprojection starting from an un-organized cloud and
    //! reserving a slot for normals
    template <srrg2_core::PointUnprojectorMode payload_offset_idx = srrg2_core::NoNormals,
              typename OutputIteratorType,
              typename SrcChannelType,
              typename... SrcChannelsRestType>
    int
    compute(OutputIteratorType dest, const SrcChannelType& src, const SrcChannelsRestType&... rest);

    //! @brief compute the organized unprojection starting from a organized cloud and reserving a
    //! slot for normals
    template <srrg2_core::PointUnprojectorMode payload_offset_idx = srrg2_core::NoNormals,
              typename PointCloudType_>
    size_t compute(const PointCloudType_& points_image_coordinates_depth_,
                   PointCloudType_& points_in_camera_frame_);

  protected:
    bool _initialization_required = true;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace srrg2_core

#include "point_unprojector_lidar3d.hpp"
