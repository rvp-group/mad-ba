#pragma once
#include "point.h"
#include "point_cloud.h"
#include "point_derived.h"
#include "point_ops.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {

  template <int Dim_, typename Scalar_>
  struct PointNormal_
    : public PointDerived_<Point_<Dim_, Scalar_>, PointDirectionField_<Scalar_, Dim_>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                = Scalar_;
    static constexpr size_t Dim = Dim_;
    using PointCoordinatesField = PointCoordinatesField_<Scalar, Dim>;
    using PointDirectionField   = PointDirectionField_<Scalar, Dim>;
    using BaseType   = PointDerived_<Point_<Dim, Scalar_>, PointDirectionField_<Scalar_, Dim_>>;
    using VectorType = Vector_<Scalar, Dim>;

    __host__ __device__ PointNormal_() {
      BaseType::coordinates().setZero();
      normal().setZero();
    }
    __host__ __device__ PointNormal_(const BaseType& other) : BaseType(other) {
    }

    __host__ __device__ inline PointNormal_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    __host__ __device__ inline typename PointDirectionField::ValueType& normal() {
      return BaseType::template value<1>();
    }
    __host__ __device__ inline const typename PointDirectionField::ValueType& normal() const {
      return BaseType::template value<1>();
    }
  };

  using PointNormal2f = PointNormal_<2, float>;
  using PointNormal2d = PointNormal_<2, double>;
  using PointNormal2i = PointNormal_<2, int>;

  using PointNormal3f = PointNormal_<3, float>;
  using PointNormal3d = PointNormal_<3, double>;
  using PointNormal3i = PointNormal_<3, int>;

  using PointNormal4f = PointNormal_<4, float>;
  using PointNormal4d = PointNormal_<4, double>;
  using PointNormal4i = PointNormal_<4, int>;

  //! @brief vector point cloud
  using PointNormal2fVectorCloud =
    PointCloud_<std::vector<PointNormal2f, Eigen::aligned_allocator<PointNormal2f>>>;
  using PointNormal2dVectorCloud =
    PointCloud_<std::vector<PointNormal2d, Eigen::aligned_allocator<PointNormal2d>>>;
  using PointNormal2iVectorCloud =
    PointCloud_<std::vector<PointNormal2i, Eigen::aligned_allocator<PointNormal2i>>>;

  using PointNormal3fVectorCloud =
    PointCloud_<std::vector<PointNormal3f, Eigen::aligned_allocator<PointNormal3f>>>;
  using PointNormal3dVectorCloud =
    PointCloud_<std::vector<PointNormal3d, Eigen::aligned_allocator<PointNormal3d>>>;
  using PointNormal3iVectorCloud =
    PointCloud_<std::vector<PointNormal3i, Eigen::aligned_allocator<PointNormal3i>>>;

  using PointNormal4fVectorCloud =
    PointCloud_<std::vector<PointNormal4f, Eigen::aligned_allocator<PointNormal4f>>>;
  using PointNormal4dVectorCloud =
    PointCloud_<std::vector<PointNormal4d, Eigen::aligned_allocator<PointNormal4d>>>;
  using PointNormal4iVectorCloud =
    PointCloud_<std::vector<PointNormal4i, Eigen::aligned_allocator<PointNormal4i>>>;

  //! @brief matrix point cloud
  using PointNormal2fMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal2f, Eigen::aligned_allocator<PointNormal2f>>>;
  using PointNormal2dMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal2d, Eigen::aligned_allocator<PointNormal2d>>>;
  using PointNormal2iMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal2i, Eigen::aligned_allocator<PointNormal2i>>>;

  using PointNormal3fMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal3f, Eigen::aligned_allocator<PointNormal3f>>>;
  using PointNormal3dMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal3d, Eigen::aligned_allocator<PointNormal3d>>>;
  using PointNormal3iMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal3i, Eigen::aligned_allocator<PointNormal3i>>>;

  using PointNormal4fMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal4f, Eigen::aligned_allocator<PointNormal4f>>>;
  using PointNormal4dMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal4d, Eigen::aligned_allocator<PointNormal4d>>>;
  using PointNormal4iMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointNormal4i, Eigen::aligned_allocator<PointNormal4i>>>;
} // namespace srrg2_core
