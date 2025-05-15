#pragma once
#include <Eigen/Eigenvalues>

#include <srrg_config/configurable.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry3d.h>

#include "point_types.h"

namespace srrg2_core {

  //! @brief base class to compute normals
  //! template arguments are for PointCloudType and
  //! the index on which you want to put the normals (in the PointType)
  template <typename PointCloudType_, int idx_>
  class NormalComputatorBase : public Configurable {
  public:
    //! @brief useful typedefs
    using PointCloudType           = PointCloudType_;
    using PointType                = typename PointCloudType::PointType;
    static constexpr int NormalIdx = idx_;

    NormalComputatorBase() {
    }
    virtual ~NormalComputatorBase() {
    }

    virtual void computeNormals(PointCloudType& point_matrix_) {
      throw std::runtime_error(
        "NormalComputatorBase<>::computeNormals | use one of the specialised "
        "NormalComputator{CrossProduct/SlidingWindow}");
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  }; // ia end class NormalComputorBase

  //! @brief specialization for organized point clouds
  template <typename PointCloudType_, int idx_>
  class NormalComputator2DCrossProduct : public NormalComputatorBase<PointCloudType_, idx_> {
  public:
    using BaseClass          = NormalComputatorBase<PointCloudType_, idx_>;
    using PointType          = typename BaseClass::PointType;
    using PointMatrixType    = Matrix_<PointType, Eigen::aligned_allocator<PointType>>;
    using Scalar             = typename BaseClass::PointType::Scalar;
    static constexpr int Dim = PointType::template TypeAt<0>::Dim;
    using VectorType         = Vector_<Scalar, Dim>;

    PARAM(PropertyUnsignedInt, row_gap, "row gap for computing cross product", 3, 0);
    PARAM(PropertyUnsignedInt, col_gap, "col gap for computing cross product", 3, 0);
    PARAM(PropertyFloat,
          squared_max_distance,
          "squared max distance between points to compute the normal",
          64.0,
          0);

    NormalComputator2DCrossProduct(){};
    virtual ~NormalComputator2DCrossProduct() {
    }

    //! @brief normal computation
    void computeNormals(PointCloudType_& point_matrix_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  }; // ia end class NormalComputator2DCrossProduct//! @brief specialization for organized point
     // clouds

  //! @brief specialization for spherical range images
  template <typename PointCloudType_, int idx_>
  class NormalComputatorSRI : public NormalComputatorBase<PointCloudType_, idx_> {
  public:
    using BaseClass          = NormalComputatorBase<PointCloudType_, idx_>;
    using PointType          = typename BaseClass::PointType;
    using Scalar             = typename BaseClass::PointType::Scalar;
    using PointMatrixType    = Matrix_<PointType, Eigen::aligned_allocator<PointType>>;
    static constexpr int Dim = PointType::template TypeAt<0>::Dim;
    using VectorType         = Vector_<Scalar, Dim>;
    using MatrixType         = MatrixN_<Scalar, Dim>;

    NormalComputatorSRI(){};
    virtual ~NormalComputatorSRI() {
    }

    //! @brief normal computation
    void computeNormals(PointCloudType_& point_matrix_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  }; // ia end class NormalComputator2DCrossProduct

  //! @brief specialization for organized point clouds
  template <typename PointCloudType_, int idx_>
  class NormalComputator2DSlidingWindow : public NormalComputatorBase<PointCloudType_, idx_> {
  public:
    using BaseClass          = NormalComputatorBase<PointCloudType_, idx_>;
    using PointType          = typename BaseClass::PointType;
    using PointMatrixType    = Matrix_<PointType, Eigen::aligned_allocator<PointType>>;
    static constexpr int Dim = PointType::template TypeAt<0>::Dim;
    using Scalar             = typename BaseClass::PointType::Scalar;
    using VectorType         = Vector_<Scalar, Dim>;
    using MatrixType         = MatrixN_<Scalar, Dim>;

    NormalComputator2DSlidingWindow(){};
    virtual ~NormalComputator2DSlidingWindow() {
    }

    PARAM(PropertyUnsignedInt, window_radius, "sliding window size", 3, 0);

    void computeNormals(PointCloudType_& point_matrix_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  //! @brief specialization for non-organized point clouds
  template <typename PointCloudType_, int idx_>
  class NormalComputator1DSlidingWindow : public NormalComputatorBase<PointCloudType_, idx_> {
  public:
    //! @brief some useful typedefs
    using BaseClass          = NormalComputatorBase<PointCloudType_, idx_>;
    using PointType          = typename BaseClass::PointType;
    static constexpr int Dim = PointType::template TypeAt<0>::Dim;
    using Scalar             = typename BaseClass::PointType::Scalar;
    using VectorType         = Vector_<Scalar, Dim>;
    using MatrixType         = MatrixN_<Scalar, Dim>;

    NormalComputator1DSlidingWindow(){};
    virtual ~NormalComputator1DSlidingWindow() {
    }

    PARAM(PropertyFloat, normal_point_distance, "max normal point distance", 0.3, 0);
    PARAM(PropertyInt, normal_min_points, "min number of points to compute a normal", 5, 0);
    PARAM(PropertyFloat, max_curvature, "max curvature", 0.2, 0);

    //! @brief compute normals in the unorganized pointcloud
    void computeNormals(PointCloudType_& point_cloud_vector_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using NormalComputator1DSlidingWindowNormal =
    NormalComputator1DSlidingWindow<PointNormal2fVectorCloud, 1>;

} // namespace srrg2_core

// implementations
#include "normal_computator.hpp"
