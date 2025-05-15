#pragma once
#include "similiarity.hpp"
#include <Eigen/Geometry>
#include <deque>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

namespace srrg2_core {

  template <typename Scalar_>
  using VectorX_ = Eigen::Matrix<Scalar_, Eigen::Dynamic, 1>;

  template <typename Scalar_, int Dim_>
  using Vector_ = Eigen::Matrix<Scalar_, Dim_, 1>;

  //! @brief statiq squared matrix
  template <typename Scalar_, int Dim_>
  using MatrixN_ = Eigen::Matrix<Scalar_, Dim_, Dim_>;

  template <typename Scalar_>
  using Vector1_ = Eigen::Matrix<Scalar_, 1, 1>;

  template <typename Scalar_>
  using Vector2_ = Eigen::Matrix<Scalar_, 2, 1>;

  template <typename Scalar_>
  using Vector3_ = Eigen::Matrix<Scalar_, 3, 1>;

  template <typename Scalar_>
  using Vector4_ = Eigen::Matrix<Scalar_, 4, 1>;

  template <typename Scalar_>
  using Vector5_ = Eigen::Matrix<Scalar_, 5, 1>;

  template <typename Scalar_>
  using Vector6_ = Eigen::Matrix<Scalar_, 6, 1>;

  template <typename Scalar_>
  using Vector7_ = Eigen::Matrix<Scalar_, 7, 1>;

  template <typename Scalar_>
  using Vector8_ = Eigen::Matrix<Scalar_, 8, 1>;

  template <typename Scalar_>
  using Vector9_ = Eigen::Matrix<Scalar_, 9, 1>;

  template <typename Scalar_>
  using Vector12_ = Eigen::Matrix<Scalar_, 12, 1>;

  template <typename Scalar_>
  using MatrixX_ = Eigen::Matrix<Scalar_, Eigen::Dynamic, Eigen::Dynamic>;

  template <typename Scalar_>
  using Matrix0_ = Eigen::Matrix<Scalar_, 0, 0>;

  template <typename Scalar_>
  using Matrix2_ = Eigen::Matrix<Scalar_, 2, 2>;

  template <typename Scalar_>
  using Matrix3_ = Eigen::Matrix<Scalar_, 3, 3>;

  template <typename Scalar_>
  using Matrix4_ = Eigen::Matrix<Scalar_, 4, 4>;

  template <typename Scalar_>
  using Matrix6_ = Eigen::Matrix<Scalar_, 6, 6>;

  template <typename Scalar_>
  using Matrix7_ = Eigen::Matrix<Scalar_, 7, 7>;

  template <typename Scalar_, int Dim>
  using Transform_ = Eigen::Transform<Scalar_, Dim, Eigen::Affine>;

  template <typename Scalar_, int Dim>
  using Isometry_ = Eigen::Transform<Scalar_, Dim, Eigen::Isometry>;

  template <typename Scalar_>
  using Isometry2_ = Isometry_<Scalar_, 2>;

  template <typename Scalar_>
  using Isometry3_ = Isometry_<Scalar_, 3>;

  template <typename Scalar_>
  using Rotation2_ = Eigen::RotationBase<Scalar_, 2>;

  template <typename Scalar_>
  using Quaternion_ = Eigen::Quaternion<Scalar_>;

  template <typename Scalar_>
  using AngleAxis_ = Eigen::AngleAxis<Scalar_>;

  using VectorXf = VectorX_<float>;
  using VectorXd = VectorX_<double>;

  using Vector1f = Vector1_<float>;
  using Vector1d = Vector1_<double>;

  using Vector2f = Vector2_<float>;
  using Vector2d = Vector2_<double>;
  using Vector2i = Vector2_<int>;

  using Vector3f  = Vector3_<float>;
  using Vector3d  = Vector3_<double>;
  using Vector3i  = Vector3_<int>;
  using Vector3uc = Vector3_<uint8_t>;

  using Vector4f = Vector4_<float>;
  using Vector4d = Vector4_<double>;

  using Vector5f = Vector5_<float>;
  using Vector5d = Vector5_<double>;

  using Vector6f = Vector6_<float>;
  using Vector6d = Vector6_<double>;

  using Vector7f = Vector7_<float>;
  using Vector7d = Vector7_<double>;

  using MatrixXf = MatrixX_<float>;
  using MatrixXd = MatrixX_<double>;

  using Matrix0f = Matrix0_<float>;
  using Matrix0d = Matrix0_<double>;

  using Matrix2f = Matrix2_<float>;
  using Matrix2d = Matrix2_<double>;

  using Matrix3f = Matrix3_<float>;
  using Matrix3d = Matrix3_<double>;

  using Matrix4f = Matrix4_<float>;
  using Matrix4d = Matrix4_<double>;

  using Matrix6f = Matrix6_<float>;
  using Matrix6d = Matrix6_<double>;

  using Isometry2f = Isometry2_<float>;
  using Isometry2d = Isometry2_<double>;

  using Isometry3f = Isometry3_<float>;
  using Isometry3d = Isometry3_<double>;

  using Quaternionf = Quaternion_<float>;
  using Quaterniond = Quaternion_<double>;

  using Rotation2f = Rotation2_<float>;
  using Rotation2d = Rotation2_<double>;

  using AngleAxisf = AngleAxis_<float>;
  using AngleAxisd = AngleAxis_<double>;

  // ldg similiarities stuff
  template <typename Scalar_>
  using Similiarity3_ = Similiarity_<Scalar_, 3>;
  using Similiarity3f = Similiarity3_<float>;
  using Similiarity3d = Similiarity3_<double>;

  template <typename Scalar_>
  using Similiarity2_ = Similiarity_<Scalar_, 2>;
  using Similiarity2f = Similiarity2_<float>;
  using Similiarity2d = Similiarity2_<double>;

  // ds TODO move?
  template <typename Scalar_>
  using Matrix1_3_ = Eigen::Matrix<Scalar_, 1, 3>;
  using Matrix1_3f = Matrix1_3_<float>;
  using Matrix1_3d = Matrix1_3_<double>;

  template <typename Scalar_>
  using Matrix2_3_ = Eigen::Matrix<Scalar_, 2, 3>;
  using Matrix2_3f = Matrix2_3_<float>;
  using Matrix2_3d = Matrix2_3_<double>;

  template <typename Scalar_>
  using Matrix2_6_ = Eigen::Matrix<Scalar_, 2, 6>;
  using Matrix2_6f = Matrix2_6_<float>;
  using Matrix2_6d = Matrix2_6_<double>;

  template <typename Scalar_>
  using Matrix3_2_ = Eigen::Matrix<Scalar_, 3, 2>;
  using Matrix3_2f = Matrix3_2_<float>;
  using Matrix3_2d = Matrix3_2_<double>;

  template <typename Scalar_>
  using Matrix3_4_ = Eigen::Matrix<Scalar_, 3, 4>;
  using Matrix3_4f = Matrix3_4_<float>;
  using Matrix3_4d = Matrix3_4_<double>;

  template <typename Scalar_>
  using Matrix3_6_ = Eigen::Matrix<Scalar_, 3, 6>;
  using Matrix3_6f = Matrix3_6_<float>;
  using Matrix3_6d = Matrix3_6_<double>;

  template <typename Scalar_>
  using Matrix4_3_ = Eigen::Matrix<Scalar_, 4, 3>;
  using Matrix4_3f = Matrix4_3_<float>;
  using Matrix4_3d = Matrix4_3_<double>;

  template <typename Scalar_>
  using Matrix4_6_ = Eigen::Matrix<Scalar_, 4, 6>;
  using Matrix4_6f = Matrix4_6_<float>;
  using Matrix4_6d = Matrix4_6_<double>;

  template <typename Scalar_>
  using Matrix5_2_ = Eigen::Matrix<Scalar_, 5, 2>;
  using Matrix5_2f = Matrix5_2_<float>;
  using Matrix5_2d = Matrix5_2_<double>;

  template <typename Scalar_>
  using Matrix5_6_ = Eigen::Matrix<Scalar_, 5, 6>;
  using Matrix5_6f = Matrix5_6_<float>;
  using Matrix5_6d = Matrix5_6_<double>;

  template <typename Scalar_>
  using Matrix6_3_ = Eigen::Matrix<Scalar_, 6, 3>;
  using Matrix6_3f = Matrix6_3_<float>;
  using Matrix6_3d = Matrix6_3_<double>;

  // ds bunch of Eigen aligned STL container types
#define DEFINE_ALIGNED_CONTAINERS(EIGEN_TYPE_)                                                   \
  using StdVectorEigen##EIGEN_TYPE_ =                                                            \
    std::vector<EIGEN_TYPE_, Eigen::aligned_allocator<EIGEN_TYPE_>>;                             \
  using StdSetEigen##EIGEN_TYPE_ = std::set<EIGEN_TYPE_, Eigen::aligned_allocator<EIGEN_TYPE_>>; \
  using StdQueueEigen##EIGEN_TYPE_ =                                                             \
    std::queue<EIGEN_TYPE_, Eigen::aligned_allocator<EIGEN_TYPE_>>;                              \
  using StdDequeEigen##EIGEN_TYPE_ = std::deque<EIGEN_TYPE_, Eigen::aligned_allocator<EIGEN_TYPE_>>
  DEFINE_ALIGNED_CONTAINERS(Vector1f);
  DEFINE_ALIGNED_CONTAINERS(Vector2f);
  DEFINE_ALIGNED_CONTAINERS(Vector3f);
  DEFINE_ALIGNED_CONTAINERS(Matrix2f);
  DEFINE_ALIGNED_CONTAINERS(Matrix3f);
  DEFINE_ALIGNED_CONTAINERS(Isometry2f);
  DEFINE_ALIGNED_CONTAINERS(Isometry3f);
  DEFINE_ALIGNED_CONTAINERS(Vector1d);
  DEFINE_ALIGNED_CONTAINERS(Vector2d);
  DEFINE_ALIGNED_CONTAINERS(Vector3d);
  DEFINE_ALIGNED_CONTAINERS(Matrix2d);
  DEFINE_ALIGNED_CONTAINERS(Matrix3d);
  DEFINE_ALIGNED_CONTAINERS(Isometry2d);
  DEFINE_ALIGNED_CONTAINERS(Isometry3d);
  DEFINE_ALIGNED_CONTAINERS(Vector2i);
  DEFINE_ALIGNED_CONTAINERS(Vector3i);

  template <typename TransformType_>
  void fixTransform(TransformType_& tf) {
    using RotationType =
      Eigen::Matrix<typename TransformType_::Scalar, TransformType_::Dim, TransformType_::Dim>;
    RotationType R = tf.linear();
    RotationType E = R.transpose() * R;
    E.diagonal().array() -= 1;
    tf.linear() -= 0.5 * R * E;
  }

  template <typename MatrixType_>
  void fixRotation(MatrixType_& R) {
    MatrixType_ E = R.transpose() * R;
    E.diagonal().array() -= 1;
    R -= 0.5 * R * E;
  }

  // bdc, please do not templatize them, otherwise we can get weird behaviour
  // i.e.
  //
  // template <typename Type_>
  // Type_ degreesToRadians (const Type_& degrees) noexcept  { return degrees *
  // static_cast<Type_> (M_PI / 180.0); }
  //
  // float theta = degreesToRadians(45); // 0
  // float theta = degreesToRadians(45.f); // 0.785398185
  //
  inline double rad2Deg(const double& rads_) noexcept {
    return (180.0 / M_PI) * rads_;
  }
  inline double deg2Rad(const double& degs_) noexcept {
    return (M_PI / 180.0) * degs_;
  }
  inline float rad2Deg(const float& rads_) noexcept {
    return (180.0f / M_PI) * rads_;
  }
  inline float deg2Rad(const float& degs_) noexcept {
    return (M_PI / 180.0f) * degs_;
  }

  template <typename MatrixType>
  using PseudoInverseType = Eigen::Matrix<typename MatrixType::Scalar,
                                          MatrixType::ColsAtCompileTime,
                                          MatrixType::RowsAtCompileTime>;

  template <typename MatrixType>
  PseudoInverseType<MatrixType> pseudoInverse(const MatrixType& a) {
    typename MatrixType::Scalar epsilon =
      std::numeric_limits<typename MatrixType::Scalar>::epsilon();
    using WorkingMatrixType = Eigen::Matrix<typename MatrixType::Scalar,
                                            Eigen::Dynamic,
                                            Eigen::Dynamic,
                                            0,
                                            MatrixType::MaxRowsAtCompileTime,
                                            MatrixType::MaxColsAtCompileTime>;
    Eigen::BDCSVD<WorkingMatrixType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon * std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<typename MatrixType::Scalar,
                  Eigen::Dynamic,
                  MatrixType::RowsAtCompileTime,
                  0,
                  Eigen::BDCSVD<MatrixType>::MaxDiagSizeAtCompileTime,
                  MatrixType::MaxRowsAtCompileTime>
      tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp   = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
  }

  template <typename VectorType>
  Eigen::Matrix<typename VectorType::Scalar, 1, VectorType::RowsAtCompileTime> Jnorm(const VectorType& p)
  {
    auto n=p.norm();
    return p.transpose()*(1./n);
  }

    template <typename VectorType>
  Eigen::Matrix<typename VectorType::Scalar,
                VectorType::RowsAtCompileTime,
                VectorType::RowsAtCompileTime> Jnormalize(const VectorType& p)
  {
    const auto n=p.norm();
    const auto in=1./n;
    const auto in3=pow(in,3);
    Eigen::Matrix<typename VectorType::Scalar,
                  VectorType::RowsAtCompileTime,
                  VectorType::RowsAtCompileTime> J;
    J.setIdentity();
    J*=in;
    J-=in3*p*p.transpose();
    return J;
  }

} // namespace srrg2_core
