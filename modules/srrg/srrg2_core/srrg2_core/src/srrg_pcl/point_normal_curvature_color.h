#pragma once
#include <srrg_data_structures/matrix.h>
#include <srrg_property/vector_data.h>

#include <srrg_pcl/point_base.h>
#include <srrg_pcl/point_cloud.h>

namespace srrg2_core {

  //! @brief point with coordinates (vector), normal (vector),
  //! curvature (float) and a rgb-color(vector).
  //! curvature can be used to compute planes from dense data
  template <int Dim_, typename Scalar_>
  struct PointNormalCurvatureColor_ : public PointBase_<Dim_,
                                                        PointCoordinatesField_<Scalar_, Dim_>,
                                                        PointDirectionField_<Scalar_, Dim_>,
                                                        PointScalarField_<Scalar_>,
                                                        PointColorField_<Scalar_, 3>> {
    //! @brief standard point usings
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar               = Scalar_;
    using BaseType             = PointBase_<Dim_,
                                PointCoordinatesField_<Scalar_, Dim_>,
                                PointDirectionField_<Scalar_, Dim_>,
                                PointScalarField_<Scalar_>,
                                PointColorField_<Scalar, 3>>;
    using PointCoordinateField = PointCoordinatesField_<Scalar_, Dim_>;
    using PointDirectionField  = PointDirectionField_<Scalar_, Dim_>;
    using PointScalarField     = PointScalarField_<Scalar_>;
    using PointColorField      = PointColorField_<Scalar_, 3>;
    static constexpr int Dim   = Dim_;
    using VectorType           = Eigen::Matrix<Scalar, Dim, 1>;
    template <typename S_>
    using PlainVectorType = typename BaseType::template PlainVectorType<S_>;

    //! @brief ctor
    PointNormalCurvatureColor_() {
      BaseType::setZero();
    }

    PointNormalCurvatureColor_(const BaseType& other) : BaseType(other) {
    }

    //! @brief operator overload
    inline PointNormalCurvatureColor_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    //! @brief defining easy accessors for fields
    inline typename PointCoordinateField::ValueType& coordinates() {
      return BaseType::template value<0>();
    }
    inline const typename PointCoordinateField::ValueType& coordinates() const {
      return BaseType::template value<0>();
    }

    inline typename PointDirectionField::ValueType& normal() {
      return BaseType::template value<1>();
    }
    inline const typename PointDirectionField::ValueType& normal() const {
      return BaseType::template value<1>();
    }

    inline typename PointScalarField::ValueType& curvature() {
      return BaseType::template value<2>();
    }
    inline const typename PointScalarField::ValueType& curvature() const {
      return BaseType::template value<2>();
    }

    inline typename PointColorField::ValueType& color() {
      return BaseType::template value<3>();
    }
    inline const typename PointColorField::ValueType& color() const {
      return BaseType::template value<3>();
    }
  };

  //! @brief basic point types
  using PointNormalCurvatureColor2f = PointNormalCurvatureColor_<2, float>;
  using PointNormalCurvatureColor2d = PointNormalCurvatureColor_<2, double>;
  using PointNormalCurvatureColor2i = PointNormalCurvatureColor_<2, int>;

  using PointNormalCurvatureColor3f = PointNormalCurvatureColor_<3, float>;
  using PointNormalCurvatureColor3d = PointNormalCurvatureColor_<3, double>;
  using PointNormalCurvatureColor3i = PointNormalCurvatureColor_<3, int>;

  using PointNormalCurvatureColor4f = PointNormalCurvatureColor_<4, float>;
  using PointNormalCurvatureColor4d = PointNormalCurvatureColor_<4, double>;
  using PointNormalCurvatureColor4i = PointNormalCurvatureColor_<4, int>;

  //! @brief vector point cloud
  using PointNormalCurvatureColor2fVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor2f,
                            Eigen::aligned_allocator<PointNormalCurvatureColor2f>>>;
  using PointNormalCurvatureColor2dVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor2d,
                            Eigen::aligned_allocator<PointNormalCurvatureColor2d>>>;
  using PointNormalCurvatureColor2iVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor2i,
                            Eigen::aligned_allocator<PointNormalCurvatureColor2i>>>;

  using PointNormalCurvatureColor3fVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor3f,
                            Eigen::aligned_allocator<PointNormalCurvatureColor3f>>>;
  using PointNormalCurvatureColor3dVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor3d,
                            Eigen::aligned_allocator<PointNormalCurvatureColor3d>>>;
  using PointNormalCurvatureColor3iVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor3i,
                            Eigen::aligned_allocator<PointNormalCurvatureColor3i>>>;

  using PointNormalCurvatureColor4fVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor4f,
                            Eigen::aligned_allocator<PointNormalCurvatureColor4f>>>;
  using PointNormalCurvatureColor4dVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor4d,
                            Eigen::aligned_allocator<PointNormalCurvatureColor4d>>>;
  using PointNormalCurvatureColor4iVectorCloud =
    PointCloud_<std::vector<PointNormalCurvatureColor4i,
                            Eigen::aligned_allocator<PointNormalCurvatureColor4i>>>;

  //! @brief matrix point cloud
  using PointNormalCurvatureColor2fMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor2f, Eigen::aligned_allocator<PointNormalCurvatureColor2f>>>;
  using PointNormalCurvatureColor2dMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor2d, Eigen::aligned_allocator<PointNormalCurvatureColor2d>>>;
  using PointNormalCurvatureColor2iMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor2i, Eigen::aligned_allocator<PointNormalCurvatureColor2i>>>;

  using PointNormalCurvatureColor3fMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor3f, Eigen::aligned_allocator<PointNormalCurvatureColor3f>>>;
  using PointNormalCurvatureColor3dMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor3d, Eigen::aligned_allocator<PointNormalCurvatureColor3d>>>;
  using PointNormalCurvatureColor3iMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor3i, Eigen::aligned_allocator<PointNormalCurvatureColor3i>>>;

  using PointNormalCurvatureColor4fMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor4f, Eigen::aligned_allocator<PointNormalCurvatureColor4f>>>;
  using PointNormalCurvatureColor4dMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor4d, Eigen::aligned_allocator<PointNormalCurvatureColor4d>>>;
  using PointNormalCurvatureColor4iMatrixCloud = PointCloud_<
    Matrix_<PointNormalCurvatureColor4i, Eigen::aligned_allocator<PointNormalCurvatureColor4i>>>;

} // namespace srrg2_core
