#pragma once
#include "point.h"
#include "point_derived.h"

namespace srrg2_core {

  template <int Dim_, typename Scalar_>
  struct PointIntensity_ : public PointDerived_<Point_<Dim_, Scalar_>, PointScalarField_<float>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                = Scalar_;
    using BaseType              = PointDerived_<Point_<Dim_, Scalar_>, PointScalarField_<float>>;
    using PointCoordinatesField = PointCoordinatesField_<Scalar_, Dim_>;
    using PointIntensityField   = PointScalarField_<float>;
    static constexpr int Dim    = Dim_;
    using VectorType            = Eigen::Matrix<Scalar, Dim, 1>;

    PointIntensity_() {
    }
    PointIntensity_(const BaseType& other) : BaseType(other) {
    }

    inline PointIntensity_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    inline typename PointIntensityField::ValueType& intensity() {
      return BaseType::template value<1>();
    }
    inline const typename PointIntensityField::ValueType& intensity() const {
      return BaseType::template value<1>();
    }
  };

  using PointIntensity2f = PointIntensity_<2, float>;
  using PointIntensity2d = PointIntensity_<2, double>;
  using PointIntensity2i = PointIntensity_<2, int>;
  using PointIntensity2fVectorCloud =
    PointCloud_<std::vector<PointIntensity2f, Eigen::aligned_allocator<PointIntensity2f>>>;
  using PointIntensity2dVectorCloud =
    PointCloud_<std::vector<PointIntensity2d, Eigen::aligned_allocator<PointIntensity2d>>>;
  using PointIntensity2iVectorCloud =
    PointCloud_<std::vector<PointIntensity2i, Eigen::aligned_allocator<PointIntensity2i>>>;

  using PointIntensity3f = PointIntensity_<3, float>;
  using PointIntensity3d = PointIntensity_<3, double>;
  using PointIntensity3i = PointIntensity_<3, int>;
  using PointIntensity3fVectorCloud =
    PointCloud_<std::vector<PointIntensity3f, Eigen::aligned_allocator<PointIntensity3f>>>;
  using PointIntensity3dVectorCloud =
    PointCloud_<std::vector<PointIntensity3d, Eigen::aligned_allocator<PointIntensity3d>>>;
  using PointIntensity3iVectorCloud =
    PointCloud_<std::vector<PointIntensity3i, Eigen::aligned_allocator<PointIntensity3i>>>;

  using PointIntensity3fMatrixCloud =
    PointCloud_<srrg2_core::Matrix_<PointIntensity3f, Eigen::aligned_allocator<PointIntensity3f>>>;
} // namespace srrg2_core
