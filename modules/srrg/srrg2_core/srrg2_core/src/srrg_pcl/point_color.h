#pragma once
#include "point_cloud.h"
#include <srrg_geometry/geometry_defs.h>
#include "point_derived.h"

namespace srrg2_core {

  template <int Dim_, typename Scalar_, int DimColor_>
  struct PointColor_ : public PointDerived_<Point_<Dim_, Scalar_> ,
                                            PointColorField_<float, DimColor_> >{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                  = Scalar_;
    using BaseType                = PointDerived_<Point_<Dim_, Scalar_> ,
                                                  PointColorField_<float, DimColor_> >;
    using PointCoordinatesField   = PointCoordinatesField_<Scalar_, Dim_>;
    using PointColorField         = PointColorField_<float, DimColor_>;
    static constexpr int Dim      = Dim_;
    static constexpr int DimColor = DimColor_;
    using VectorType              = Eigen::Matrix<Scalar, Dim, 1>;

    PointColor_() {
    }
    PointColor_(const BaseType& other) : BaseType(other) {
    }

    inline PointColor_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    inline typename PointColorField::ValueType& color() {
      return BaseType::template value<1>();
    }
    inline const typename PointColorField::ValueType& color() const {
      return BaseType::template value<1>();
    }
  };

  using PointColor2f = PointColor_<2, float, 3>;
  using PointColor2d = PointColor_<2, double, 3>;
  using PointColor2i = PointColor_<2, int, 3>;

  using PointColor3f = PointColor_<3, float, 3>;
  using PointColor3d = PointColor_<3, double, 3>;
  using PointColor3i = PointColor_<3, int, 3>;

  using PointColor4f = PointColor_<4, float, 3>;
  using PointColor4d = PointColor_<4, double, 3>;
  using PointColor4i = PointColor_<4, int, 3>;

  //! @brief vector point cloud
  using PointColor2fVectorCloud = PointCloud_<
    std::vector<PointColor2f, Eigen::aligned_allocator<PointColor2f>>>;
  using PointColor2dVectorCloud = PointCloud_<
    std::vector<PointColor2d, Eigen::aligned_allocator<PointColor2d>>>;
  using PointColor2iVectorCloud = PointCloud_<
    std::vector<PointColor2i, Eigen::aligned_allocator<PointColor2i>>>;

  using PointColor3fVectorCloud = PointCloud_<
    std::vector<PointColor3f, Eigen::aligned_allocator<PointColor3f>>>;
  using PointColor3dVectorCloud = PointCloud_<
    std::vector<PointColor3d, Eigen::aligned_allocator<PointColor3d>>>;
  using PointColor3iVectorCloud = PointCloud_<
    std::vector<PointColor3i, Eigen::aligned_allocator<PointColor3i>>>;

  using PointColor4fVectorCloud = PointCloud_<
    std::vector<PointColor4f, Eigen::aligned_allocator<PointColor4f>>>;
  using PointColor4dVectorCloud = PointCloud_<
    std::vector<PointColor4d, Eigen::aligned_allocator<PointColor4d>>>;
  using PointColor4iVectorCloud = PointCloud_<
    std::vector<PointColor4i, Eigen::aligned_allocator<PointColor4i>>>;

  //! @brief matrix point cloud
  using PointColor2fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor2f, Eigen::aligned_allocator<PointColor2f>>>;
  using PointColor2dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor2d, Eigen::aligned_allocator<PointColor2d>>>;
  using PointColor2iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor2i, Eigen::aligned_allocator<PointColor2i>>>;

  using PointColor3fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor3f, Eigen::aligned_allocator<PointColor3f>>>;
  using PointColor3dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor3d, Eigen::aligned_allocator<PointColor3d>>>;
  using PointColor3iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor3i, Eigen::aligned_allocator<PointColor3i>>>;

  using PointColor4fMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor4f, Eigen::aligned_allocator<PointColor4f>>>;
  using PointColor4dMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor4d, Eigen::aligned_allocator<PointColor4d>>>;
  using PointColor4iMatrixCloud = PointCloud_<
    srrg2_core::Matrix_<PointColor4i, Eigen::aligned_allocator<PointColor4i>>>;
} // namespace srrg2_core
