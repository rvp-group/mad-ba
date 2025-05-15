#pragma once
#include "srrg_config/property_configurable.h"
#include "srrg_pcl/lidar_3d_utils/lidar_3d_lookup_projector.h"
#include "srrg_pcl/point_projector.h"

namespace srrg2_core {

  template <srrg2_lidar3d_utils::LIDAR_TYPE lidar3d_type_, typename PointCloudType_>
  class PointProjectorLidar3D_
    : public srrg2_core::PointProjectorBase_<PointCloudType_>,
      public srrg2_lidar3d_utils::Lidar3DLookupProjector_<lidar3d_type_> {
  public:
    using ThisType                   = PointProjectorLidar3D_<lidar3d_type_, PointCloudType_>;
    using BaseType                   = srrg2_core::PointProjectorBase_<PointCloudType_>;
    using LidarProjectorBaseType     = srrg2_lidar3d_utils::Lidar3DLookupProjector_<lidar3d_type_>;
    using PointCloudType             = PointCloudType_;
    using PointType                  = typename PointCloudType::PointType;
    using IteratorType               = typename PointCloudType::iterator;
    using ConstIteratorType          = typename PointCloudType::const_iterator;
    using IsometryType               = srrg2_core::Isometry_<float, PointType::GeometryDim>;
    using Lidar3DSensorType          = srrg2_lidar3d_utils::Lidar3DSensor_<lidar3d_type_>;
    static constexpr int GeometryDim = PointType::GeometryDim;
    static constexpr srrg2_lidar3d_utils::LIDAR_TYPE LidarType = lidar3d_type_;

    PARAM(srrg2_core::PropertyUnsignedInt,
          num_columns,
          "number of columns in the projected image of the lidar scan",
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

    int
    compute(typename BaseType::TargetMatrixType& target_, IteratorType begin_, IteratorType end_);

  protected:
    bool _initialization_required = true;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_core

// bdc ugly template definitions
#include "point_projector_lidar3d.hpp"
