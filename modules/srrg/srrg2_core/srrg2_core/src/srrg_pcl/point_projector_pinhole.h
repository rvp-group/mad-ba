#pragma once
#include <limits>

#include "srrg_config/configurable.h"
#include "srrg_data_structures/matrix.h"

#include "point_projector.h"

namespace srrg2_core {

  template <typename PointCloudType_>
  struct PointProjectorPinhole_
    : public PointProjector_<TRANSFORM_CLASS::PinholeProjection,
                             PointCloudType_> {
  public:
    using ThisType = PointProjectorPinhole_<PointCloudType_>;
    using BaseType =
      PointProjector_<TRANSFORM_CLASS::PinholeProjection, PointCloudType_>;
    using PointCloudType             = typename BaseType::PointCloudType;
    using PointType                  = typename PointCloudType::PointType;
    using IteratorType               = typename PointCloudType::iterator;
    using ConstIteratorType          = typename PointCloudType::const_iterator;
    static constexpr int GeometryDim = PointType::GeometryDim;
    using IsometryType               = Isometry_<float, GeometryDim>;
    using ProjectionType             = Transform_<float, GeometryDim>;
    using ProjectedMatrixType        = typename BaseType::ProjectedMatrixType;
    using TransformedMatrixType      = typename BaseType::ProjectedMatrixType;
    using DepthMatrixType            = Matrix_<float>;
    using IteratorMatrixType         = Matrix_<IteratorType>;
    using ConstIteratorMatrixType    = Matrix_<ConstIteratorType>;
    using IndexMatrixType            = Matrix_<int>;

    PointProjectorPinhole_() {
    }

    void initCameraMatrix() override {
      // ds do not overwrite a valid camera matrix with the identity
      // ds TODO infer proper camera matrix from (some?) configuration
      //      this->_camera_matrix.setIdentity();
    }

    void adjustProjectionMatrix(ProjectionType& projection_matrix) override {
      projection_matrix.setIdentity();
      projection_matrix.linear() = this->_camera_matrix;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_core
