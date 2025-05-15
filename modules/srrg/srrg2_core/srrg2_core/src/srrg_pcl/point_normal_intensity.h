#pragma once
#include "point_normal.h"

namespace srrg2_core {

  template <int Dim_, typename Scalar_>
  struct PointNormalIntensity_
    : public  PointDerived_<PointNormal_<Dim_, Scalar_> ,
                            PointScalarField_<float> > {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                = Scalar_;
    using BaseType              = PointDerived_<PointNormal_<Dim_, Scalar_> ,
                                                PointScalarField_<float> >;
    
    using PointCoordinatesField = PointCoordinatesField_<Scalar_, Dim_>;
    using PointDirectionField   = PointDirectionField_<Scalar_, Dim_>;
    using PointIntensityField   = PointScalarField_<float>;
    static constexpr int Dim    = Dim_;
    using VectorType            = Eigen::Matrix<Scalar, Dim, 1>;

    __host__ __device__ PointNormalIntensity_() {
    }
    __host__ __device__ PointNormalIntensity_(const BaseType& other) : BaseType(other) {
    }

    __host__ __device__ inline PointNormalIntensity_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    __host__ __device__ inline typename PointIntensityField::ValueType& intensity() {
      return BaseType::template value<2>();
    }
    
    __host__ __device__ inline const typename PointIntensityField::ValueType& intensity() const {
      return BaseType::template value<2>();
    }
  };

  using PointNormalIntensity2f = PointNormalIntensity_<2, float>;
  using PointNormalIntensity2d = PointNormalIntensity_<2, double>;
  using PointNormalIntensity2i = PointNormalIntensity_<2, int>;
  using PointNormalIntensity2fVectorCloud =
    PointCloud_<std::vector<PointNormalIntensity2f,
                            Eigen::aligned_allocator<PointNormalIntensity2f>>>;
  using PointNormalIntensity2dVectorCloud =
    PointCloud_<std::vector<PointNormalIntensity2d,
                            Eigen::aligned_allocator<PointNormalIntensity2d>>>;
  using PointNormalIntensity2iVectorCloud =
    PointCloud_<std::vector<PointNormalIntensity2i,
                            Eigen::aligned_allocator<PointNormalIntensity2i>>>;

  using PointNormalIntensity3f = PointNormalIntensity_<3, float>;
  using PointNormalIntensity3d = PointNormalIntensity_<3, double>;
  using PointNormalIntensity3i = PointNormalIntensity_<3, int>;
  using PointNormalIntensity3fVectorCloud =
    PointCloud_<std::vector<PointNormalIntensity3f,
                            Eigen::aligned_allocator<PointNormalIntensity3f>>>;
  using PointNormalIntensity3dVectorCloud =
    PointCloud_<std::vector<PointNormalIntensity3d,
                            Eigen::aligned_allocator<PointNormalIntensity3d>>>;
  using PointNormalIntensity3iVectorCloud =
    PointCloud_<std::vector<PointNormalIntensity3i,
                            Eigen::aligned_allocator<PointNormalIntensity3i>>>;
} // namespace srrg2_core
