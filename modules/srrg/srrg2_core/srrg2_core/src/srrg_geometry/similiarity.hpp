#pragma once
#include <Eigen/Core>
#include <iostream>

namespace srrg2_core {
  //! @brief ldg implements similiarity class eigen illnesses style
  template <typename Scalar_, int Dim_>
  class Similiarity_ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int Dim       = Dim_;
    static constexpr int MatrixDim = Dim + 1;
    using Index                    = Eigen::Index;
    using Scalar                   = Scalar_;
    using ThisType                 = Similiarity_<Scalar, Dim>;
    using MatrixType               = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;
    using VectorType               = Eigen::Matrix<Scalar, Dim, 1>;
    using ConstMatrixType          = const MatrixType;
    using LinearType               = Eigen::Matrix<Scalar, Dim, Dim>;
    using ConstLinearType          = const LinearType;
    using TranslationType          = Eigen::Matrix<Scalar, Dim, 1>;
    using ConstTranslationType     = const TranslationType;
    // ldg blocks in super fancy eigen way
    using LinearBlock           = Eigen::Block<MatrixType, Dim, Dim>;
    using ConstLinearBlock      = const Eigen::Block<ConstMatrixType, Dim, Dim>;
    using TranslationBlock      = Eigen::Block<MatrixType, Dim, 1>;
    using ConstTranslationBlock = const Eigen::Block<ConstMatrixType, Dim, 1>;

    //! @brief ldg constructing similiarity to identity
    Similiarity_() {
      _matrix.setIdentity();
    }

    //! @brief ldg copy construction, allows direct initialization
    Similiarity_(const Similiarity_& other) {
      _matrix.setIdentity();
      this->linear()         = other.linear();
      this->translation()    = other.translation();
      this->inverseScaling() = other.inverseScaling();
    }

    static ThisType Identity() {
      return ThisType();
    }

    //! @brief ldg accessor matrix type
    ConstMatrixType& matrix() const {
      return _matrix;
    }

    // ! @brief ldg setter matrix type, eigen style
    MatrixType& matrix() {
      return _matrix;
    }

    //! @brief ldg accessor to rotational part
    ConstLinearBlock linear() const {
      return ConstLinearBlock(_matrix, 0, 0);
    }

    //! @brief ldg setter to rotational part
    LinearBlock linear() {
      return LinearBlock(_matrix, 0, 0);
    }

    //! @brief ldg accessor to rotational part
    ConstLinearBlock rotation() const {
      return linear();
    }

    //! @brief ldg setter to rotational part
    LinearBlock rotation() {
      return linear();
    }

    //! @brief ldg accessor to translational part
    TranslationBlock translation() {
      return TranslationBlock(_matrix, 0, Dim);
    }

    //! @brief ldg setter to translational part
    ConstTranslationBlock translation() const {
      return ConstTranslationBlock(_matrix, 0, Dim);
    }

    //! @brief ldg accessor to inverse scaling value, scalar
    const Scalar& inverseScaling() const {
      return _matrix.coeffRef(Dim, Dim);
    }

    //! @brief ldg setter to inverse scaling value, scalar
    Scalar& inverseScaling() {
      return _matrix.coeffRef(Dim, Dim);
    }

    //! @brief ldg set matrix to indentity
    void setIdentity() {
      _matrix.setIdentity();
    }

    //! @brief ldg returns a inverse of a similiarity
    ThisType inverse() const {
      ThisType inverse_sim;
      inverse_sim.setIdentity();
      inverse_sim.linear() = this->linear().transpose();
      inverse_sim.translation() =
        -Scalar(1) / this->inverseScaling() * (inverse_sim.linear() * this->translation());
      inverse_sim.inverseScaling() = Scalar(1) / this->inverseScaling();
      return inverse_sim;
    }

    //! @brief ldg multiply operator between similiarities
    ThisType operator*(const ThisType& other) const {
      ThisType result;
      result.setIdentity();
      ConstLinearType& R      = other.linear();
      ConstTranslationType& t = other.translation();
      const Scalar& is        = other.inverseScaling();
      result.linear()         = this->linear() * R;
      result.translation()    = this->linear() * t + is * this->translation();
      result.inverseScaling() = is * this->inverseScaling();
      return result;
    }

    //! @brief ldg multiply operator similiarity * vector
    VectorType operator*(const VectorType& other) const {
      VectorType result;
      result = Scalar(1) / this->inverseScaling() * (this->linear() * other + this->translation());
      return result;
    }

    //! @brief ldg equal operator, copy similiarities
    ThisType& operator=(const ThisType& other) {
      this->linear()         = other.linear();
      this->translation()    = other.translation();
      this->inverseScaling() = other.inverseScaling();
      return *this;
    }

    //! @brief ldg multiply operator between similiarities, write result on current sim
    ThisType& operator*=(const ThisType& other) {
      *this = *this * other;
      return *this;
    }

    Scalar& operator()(Index row, Index col) {
      return _matrix(row, col);
    }

    Scalar operator()(Index row, Index col) const {
      return _matrix(row, col);
    }

    Index rows() const {
      return _matrix.rows();
    }

    Index cols() const {
      return _matrix.cols();
    }

  protected:
    MatrixType _matrix;
  };

} // namespace srrg2_core
