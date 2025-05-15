#pragma once
#include "point_cloud.h"
#include <cstdint>
#include <srrg_geometry/geometry_defs.h>
#include "point_derived.h"
#include "point_default_field.h"

namespace srrg2_core {
  using PointId=int64_t;
    
  template <int Dim_, typename Scalar_>
  struct PointId_ : public PointDerived_<Point_<Dim_, Scalar_> ,
                                            PointDefaultField_<PointId> >{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                  = Scalar_;
    using BaseType                = PointDerived_<Point_<Dim_, Scalar_> ,
                                                  PointDefaultField_<PointId> >;
    using PointCoordinatesField   = PointCoordinatesField_<Scalar_, Dim_>;
    using PointIdField            = PointDefaultField_<PointId>;
    static constexpr int Dim      = Dim_;
    using VectorType              = Eigen::Matrix<Scalar, Dim, 1>;

    PointId_() {
    }
    PointId_(const BaseType& other) : BaseType(other) {
    }

    inline PointId_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    inline typename PointIdField::ValueType& id() {
      return BaseType::template value<1>();
    }
    inline const typename PointIdField::ValueType& id() const {
      return BaseType::template value<1>();
    }
  };

  using PointId2f = PointId_<2, float>;
  using PointId2d = PointId_<2, double>;
  using PointId2i = PointId_<2, int>;

  using PointId3f = PointId_<3, float>;
  using PointId3d = PointId_<3, double>;
  using PointId3i = PointId_<3, int>;

  using PointId4f = PointId_<4, float>;
  using PointId4d = PointId_<4, double>;
  using PointId4i = PointId_<4, int>;

  //! @brief vector point cloud
  using PointId2fVectorCloud = PointCloud_<
    std::vector<PointId2f, Eigen::aligned_allocator<PointId2f>>>;
  using PointId2dVectorCloud = PointCloud_<
    std::vector<PointId2d, Eigen::aligned_allocator<PointId2d>>>;
  using PointId2iVectorCloud = PointCloud_<
    std::vector<PointId2i, Eigen::aligned_allocator<PointId2i>>>;

  using PointId3fVectorCloud = PointCloud_<
    std::vector<PointId3f, Eigen::aligned_allocator<PointId3f>>>;
  using PointId3dVectorCloud = PointCloud_<
    std::vector<PointId3d, Eigen::aligned_allocator<PointId3d>>>;
  using PointId3iVectorCloud = PointCloud_<
    std::vector<PointId3i, Eigen::aligned_allocator<PointId3i>>>;

  using PointId4fVectorCloud = PointCloud_<
    std::vector<PointId4f, Eigen::aligned_allocator<PointId4f>>>;
  using PointId4dVectorCloud = PointCloud_<
    std::vector<PointId4d, Eigen::aligned_allocator<PointId4d>>>;
  using PointId4iVectorCloud = PointCloud_<
    std::vector<PointId4i, Eigen::aligned_allocator<PointId4i>>>;

  //! @brief matrix point cloud
  using PointId2fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId2f, Eigen::aligned_allocator<PointId2f>>>;
  using PointId2dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId2d, Eigen::aligned_allocator<PointId2d>>>;
  using PointId2iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId2i, Eigen::aligned_allocator<PointId2i>>>;

  using PointId3fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId3f, Eigen::aligned_allocator<PointId3f>>>;
  using PointId3dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId3d, Eigen::aligned_allocator<PointId3d>>>;
  using PointId3iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId3i, Eigen::aligned_allocator<PointId3i>>>;

  using PointId4fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId4f, Eigen::aligned_allocator<PointId4f>>>;
  using PointId4dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId4d, Eigen::aligned_allocator<PointId4d>>>;
  using PointId4iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointId4i, Eigen::aligned_allocator<PointId4i>>>;
} // namespace srrg2_core
