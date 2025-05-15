#pragma once
#include "point_normal.h"

namespace srrg2_core {

  //! @brief point with coordinates (vector), normal (vector)
  //! and curvature (float). last one can be used to compute planes
  //! from dense data
  template <int Dim_, typename Scalar_>
  struct PointNormalCurvature_
    : public  PointDerived_<PointNormal_<Dim_, Scalar_> ,
                            PointScalarField_<float> > {
    //! @brief standard point usings
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Scalar               = Scalar_;
    using BaseType             = PointDerived_<PointNormal_<Dim_, Scalar_> ,
                                               PointScalarField_<float> >;
    using PointCoordinateField = PointCoordinatesField_<Scalar_, Dim_>;
    using PointDirectionField  = PointDirectionField_<Scalar_, Dim_>;
    using PointScalarField     = PointScalarField_<Scalar_>;
    static constexpr int Dim   = Dim_;
    using VectorType           = Eigen::Matrix<Scalar, Dim, 1>;

    //! @brief ctor
    PointNormalCurvature_() {
      BaseType::setZero();
    }

    PointNormalCurvature_(const BaseType& other) : BaseType(other) {
    }

    //! @brief operator overload
    inline PointNormalCurvature_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    inline typename PointScalarField::ValueType& curvature() {
      return BaseType::template value<2>();
    }
    inline const typename PointScalarField::ValueType& curvature() const {
      return BaseType::template value<2>();
    }
  };

  //! @brief basic point types
  using PointNormalCurvature2f = PointNormalCurvature_<2, float>;
  using PointNormalCurvature2d = PointNormalCurvature_<2, double>;
  using PointNormalCurvature2i = PointNormalCurvature_<2, int>;

  using PointNormalCurvature3f = PointNormalCurvature_<3, float>;
  using PointNormalCurvature3d = PointNormalCurvature_<3, double>;
  using PointNormalCurvature3i = PointNormalCurvature_<3, int>;

  using PointNormalCurvature4f = PointNormalCurvature_<4, float>;
  using PointNormalCurvature4d = PointNormalCurvature_<4, double>;
  using PointNormalCurvature4i = PointNormalCurvature_<4, int>;

  //! @brief vector point cloud
  using PointNormalCurvature2fVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature2f,
                            Eigen::aligned_allocator<PointNormalCurvature2f>>>;
  using PointNormalCurvature2dVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature2d,
                            Eigen::aligned_allocator<PointNormalCurvature2d>>>;
  using PointNormalCurvature2iVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature2i,
                            Eigen::aligned_allocator<PointNormalCurvature2i>>>;

  using PointNormalCurvature3fVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature3f,
                            Eigen::aligned_allocator<PointNormalCurvature3f>>>;
  using PointNormalCurvature3dVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature3d,
                            Eigen::aligned_allocator<PointNormalCurvature3d>>>;
  using PointNormalCurvature3iVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature3i,
                            Eigen::aligned_allocator<PointNormalCurvature3i>>>;

  using PointNormalCurvature4fVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature4f,
                            Eigen::aligned_allocator<PointNormalCurvature4f>>>;
  using PointNormalCurvature4dVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature4d,
                            Eigen::aligned_allocator<PointNormalCurvature4d>>>;
  using PointNormalCurvature4iVectorCloud =
    PointCloud_<std::vector<PointNormalCurvature4i,
                            Eigen::aligned_allocator<PointNormalCurvature4i>>>;

  //! @brief matrix point cloud
  using PointNormalCurvature2fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature2f,
                        Eigen::aligned_allocator<PointNormalCurvature2f>>>;
  using PointNormalCurvature2dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature2d,
                        Eigen::aligned_allocator<PointNormalCurvature2d>>>;
  using PointNormalCurvature2iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature2i,
                        Eigen::aligned_allocator<PointNormalCurvature2i>>>;

  using PointNormalCurvature3fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature3f,
                        Eigen::aligned_allocator<PointNormalCurvature3f>>>;
  using PointNormalCurvature3dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature3d,
                        Eigen::aligned_allocator<PointNormalCurvature3d>>>;
  using PointNormalCurvature3iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature3i,
                        Eigen::aligned_allocator<PointNormalCurvature3i>>>;

  using PointNormalCurvature4fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature4f,
                        Eigen::aligned_allocator<PointNormalCurvature4f>>>;
  using PointNormalCurvature4dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature4d,
                        Eigen::aligned_allocator<PointNormalCurvature4d>>>;
  using PointNormalCurvature4iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalCurvature4i,
                        Eigen::aligned_allocator<PointNormalCurvature4i>>>;

} // namespace srrg2_core
