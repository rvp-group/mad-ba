#pragma once
#include "srrg_geometry/geometry_defs.h"

#include "point_normal.h"

namespace srrg2_core {

  template <int Dim_, typename Scalar_, int DimColor_>
  struct PointNormalColor_
    : public PointDerived_<PointNormal_<Dim_,Scalar_>,
                           PointColorField_<float, DimColor_> > {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                  = Scalar_;
    using BaseType                = PointDerived_<PointNormal_<Dim_,Scalar_>,
                                                  PointColorField_<float, DimColor_> >;
    using PointCoordinatesField   = PointCoordinatesField_<Scalar_, Dim_>;
    using PointDirectionField     = PointDirectionField_<Scalar_, Dim_>;
    using PointColorField         = PointColorField_<float, DimColor_>;
    static constexpr int Dim      = Dim_;
    static constexpr int DimColor = DimColor_;
    using VectorType              = Eigen::Matrix<Scalar, Dim, 1>;

    PointNormalColor_() {
    }
    PointNormalColor_(const BaseType& other) : BaseType(other) {
    }

    inline PointNormalColor_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    inline typename PointColorField::ValueType& color() {
      return BaseType::template value<2>();
    }
    inline const typename PointColorField::ValueType& color() const {
      return BaseType::template value<2>();
    }
  };

  using PointNormalColor2f = PointNormalColor_<2, float, 3>;
  using PointNormalColor2d = PointNormalColor_<2, double, 3>;
  using PointNormalColor2i = PointNormalColor_<2, int, 3>;

  using PointNormalColor3f = PointNormalColor_<3, float, 3>;
  using PointNormalColor3d = PointNormalColor_<3, double, 3>;
  using PointNormalColor3i = PointNormalColor_<3, int, 3>;

  using PointNormalColor4f = PointNormalColor_<4, float, 3>;
  using PointNormalColor4d = PointNormalColor_<4, double, 3>;
  using PointNormalColor4i = PointNormalColor_<4, int, 3>;

  //! @brief vector point cloud
  using PointNormalColor2fVectorCloud =
    PointCloud_<std::vector<PointNormalColor2f,
                            Eigen::aligned_allocator<PointNormalColor2f>>>;
  using PointNormalColor2dVectorCloud =
    PointCloud_<std::vector<PointNormalColor2d,
                            Eigen::aligned_allocator<PointNormalColor2d>>>;
  using PointNormalColor2iVectorCloud =
    PointCloud_<std::vector<PointNormalColor2i,
                            Eigen::aligned_allocator<PointNormalColor2i>>>;

  using PointNormalColor3fVectorCloud =
    PointCloud_<std::vector<PointNormalColor3f,
                            Eigen::aligned_allocator<PointNormalColor3f>>>;
  using PointNormalColor3dVectorCloud =
    PointCloud_<std::vector<PointNormalColor3d,
                            Eigen::aligned_allocator<PointNormalColor3d>>>;
  using PointNormalColor3iVectorCloud =
    PointCloud_<std::vector<PointNormalColor3i,
                            Eigen::aligned_allocator<PointNormalColor3i>>>;

  using PointNormalColor4fVectorCloud =
    PointCloud_<std::vector<PointNormalColor4f,
                            Eigen::aligned_allocator<PointNormalColor4f>>>;
  using PointNormalColor4dVectorCloud =
    PointCloud_<std::vector<PointNormalColor4d,
                            Eigen::aligned_allocator<PointNormalColor4d>>>;
  using PointNormalColor4iVectorCloud =
    PointCloud_<std::vector<PointNormalColor4i,
                            Eigen::aligned_allocator<PointNormalColor4i>>>;

  //! @brief matrix point cloud
  using PointNormalColor2fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor2f,
                        Eigen::aligned_allocator<PointNormalColor2f>>>;
  using PointNormalColor2dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor2d,
                        Eigen::aligned_allocator<PointNormalColor2d>>>;
  using PointNormalColor2iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor2i,
                        Eigen::aligned_allocator<PointNormalColor2i>>>;

  using PointNormalColor3fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor3f,
                        Eigen::aligned_allocator<PointNormalColor3f>>>;
  using PointNormalColor3dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor3d,
                        Eigen::aligned_allocator<PointNormalColor3d>>>;
  using PointNormalColor3iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor3i,
                        Eigen::aligned_allocator<PointNormalColor3i>>>;

  using PointNormalColor4fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor4f,
                        Eigen::aligned_allocator<PointNormalColor4f>>>;
  using PointNormalColor4dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor4d,
                        Eigen::aligned_allocator<PointNormalColor4d>>>;
  using PointNormalColor4iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointNormalColor4i,
                        Eigen::aligned_allocator<PointNormalColor4i>>>;

} // namespace srrg2_core
