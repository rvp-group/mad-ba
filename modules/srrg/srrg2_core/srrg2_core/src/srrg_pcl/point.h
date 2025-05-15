#pragma once
#include "point_cloud.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {

  template <int Dim_, typename Scalar_>
  struct Point_
    : public PointBase_<Dim_, PointCoordinatesField_<Scalar_, Dim_>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Scalar   = Scalar_;
    using BaseType = PointBase_<Dim_, PointCoordinatesField_<Scalar_, Dim_>>;
    using PointCoordinatesField = PointCoordinatesField_<Scalar_, Dim_>;
    static constexpr int Dim    = Dim_;
    using VectorType            = Eigen::Matrix<Scalar, Dim, 1>;

    __host__ __device__ Point_() {
    }

    __host__ __device__ Point_(const BaseType& other) : BaseType(other) {
    }

    __host__ __device__ inline Point_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    __host__ __device__ inline typename PointCoordinatesField::ValueType& coordinates() {
      return BaseType::template value<0>();
    }
    __host__ __device__ inline const typename PointCoordinatesField::ValueType&
    coordinates() const {
      return BaseType::template value<0>();
    }
  };

  template <typename Scalar_>
  using Point2_ = Point_<2, Scalar_>;

  using Point2f = Point2_<float>;
  using Point2d = Point2_<double>;
  using Point2i = Point2_<int>;

  template <typename Scalar_>
  using Point3_ = Point_<3, Scalar_>;

  using Point3f = Point3_<float>;
  using Point3d = Point3_<double>;
  using Point3i = Point3_<int>;

  template <typename Scalar_>
  using Point4_ = Point_<4, Scalar_>;

  using Point4f = Point4_<float>;
  using Point4d = Point4_<double>;
  using Point4i = Point4_<int>;

  //! @brief vector point cloud
  using Point2fVectorCloud =
    PointCloud_<std::vector<Point2f, Eigen::aligned_allocator<Point2f>>>;
  using Point2dVectorCloud =
    PointCloud_<std::vector<Point2d, Eigen::aligned_allocator<Point2d>>>;
  using Point2iVectorCloud =
    PointCloud_<std::vector<Point2i, Eigen::aligned_allocator<Point2i>>>;

  using Point3fVectorCloud =
    PointCloud_<std::vector<Point3f, Eigen::aligned_allocator<Point3f>>>;
  using Point3dVectorCloud =
    PointCloud_<std::vector<Point3d, Eigen::aligned_allocator<Point3d>>>;
  using Point3iVectorCloud =
    PointCloud_<std::vector<Point3i, Eigen::aligned_allocator<Point3i>>>;

  using Point4fVectorCloud =
    PointCloud_<std::vector<Point4f, Eigen::aligned_allocator<Point4f>>>;
  using Point4dVectorCloud =
    PointCloud_<std::vector<Point4d, Eigen::aligned_allocator<Point4d>>>;
  using Point4iVectorCloud =
    PointCloud_<std::vector<Point4i, Eigen::aligned_allocator<Point4i>>>;

  //! @brief matrix point cloud
  using Point2fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point2f, Eigen::aligned_allocator<Point2f>>>;
  using Point2dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point2d, Eigen::aligned_allocator<Point2d>>>;
  using Point2iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point2i, Eigen::aligned_allocator<Point2i>>>;

  using Point3fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point3f, Eigen::aligned_allocator<Point3f>>>;
  using Point3dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point3d, Eigen::aligned_allocator<Point3d>>>;
  using Point3iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point3i, Eigen::aligned_allocator<Point3i>>>;

  using Point4fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point4f, Eigen::aligned_allocator<Point4f>>>;
  using Point4dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point4d, Eigen::aligned_allocator<Point4d>>>;
  using Point4iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<Point4i, Eigen::aligned_allocator<Point4i>>>;

} // namespace srrg2_core
