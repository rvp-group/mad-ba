#pragma once
#include <set>
#include <vector>

#include "srrg_geometry/geometry3d.h"

namespace srrg2_core {

  class MatchableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief type of matchable
    enum Type : int { Point = 0, Line = 1, Plane = 2, Surfel = 3, Unknown = 4 };

    //! @brief constructor based on type
    MatchableBase(const Type& type_) : _type(type_) {
    }

    //! @brief type accessor
    inline const Type& type() const {
      return _type;
    }

  protected:
    //! @brief type of the matchable
    Type _type;
  };

  //! @brief actual matchable class. this represents a mathemagical
  //!        type - like an Isometry3 or a float - and, thus, here
  //!        there are only stuff that can be useful at the mathematical
  //!        level. No drawing function, no parameters for the age
  //!        and other stuff like that.
  template <typename Scalar_>
  class Matchable_ : public MatchableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Scalar             = Scalar_;
    using VectorType         = Vector3_<Scalar>;
    using MatrixType         = Matrix3_<Scalar>;
    static constexpr int Dim = VectorType::RowsAtCompileTime + // ia origin
                               MatrixType::RowsAtCompileTime + // ia direction
                               1;                              // ia type
    using FullVectorType = Vector_<Scalar, Dim>;

    //! @brief empty ctor - sets everything to zero;
    Matchable_() :
      MatchableBase(Type::Point),
      _origin(VectorType::Zero()),
      _rotation(MatrixType::Identity()) {
      resetOmegaFromType();
    }

    //! @brief ctor from origin and rotation matrix
    Matchable_(const Type& type_,
               const VectorType& origin_   = VectorType::Zero(),
               const MatrixType& rotation_ = MatrixType::Identity()) :
      MatchableBase(type_),
      _origin(origin_),
      _rotation(rotation_) {
      // ia reset the omega
      resetOmegaFromType();
    }

    //! @brief ctor from vectorized matchable
    Matchable_(const FullVectorType& vector_) : MatchableBase(vector_(0)) {
      resetOmegaFromType();
      fromVector(vector_);
    }

    //! @brief copy ctor
    Matchable_(const Matchable_<Scalar_>& other_) :
      MatchableBase(other_._type),
      _origin(other_._origin),
      _rotation(other_._rotation),
      _activation_matrix(other_._activation_matrix) {
    }

    //! @brief the only way to change the type of the matchable
    inline void setType(const Type& type_) {
      _type = type_;
      resetOmegaFromType();
    }

    //! @brief returns a matchable in a vector form [type; origin; direction_v]'
    inline const FullVectorType toVector() const {
      FullVectorType vector             = FullVectorType::Zero();
      vector(0)                         = _type;
      vector.template block<3, 1>(1, 0) = _origin;
      vector.template block<3, 1>(4, 0) = _rotation.col(0);
      return vector;
    }

    //! @brief populates the matchable from [type; origin; direction_v]'.
    inline void fromVector(const FullVectorType& vector_) {
      // ia reset the type
      _type = (MatchableBase::Type) vector_(0);

      resetOmegaFromType();

      // ia set origin and rotation matrix
      _origin = vector_.template block<3, 1>(1, 0);
      setDirection(vector_.template block<3, 1>(4, 0));
    }

    //! @brief direction vector is the 1st column of rotation matrix
    inline const VectorType direction() const {
      return (VectorType) _rotation.col(0);
    }

    //! @brief compute and set the rotation matrix from the direction of
    //!        the normal - computed as Rz*Rx*Ry
    void setDirection(const VectorType& direction_);

    //! @brief apply a transform to the matchable
    inline Matchable_<Scalar> transform(const Isometry3_<Scalar>& isometry_) const {
      Matchable_<Scalar> dest(*this);
      dest._origin = isometry_ * _origin;
      if (_type != Type::Point) {
        dest._rotation = isometry_.linear() * _rotation;
      }
      return dest;
    }

    //! @brief apply a transform to this
    inline void transformInPlace(const Isometry3_<Scalar>& isometry_) {
      _origin = isometry_ * _origin;
      if (_type != Type::Point) {
        _rotation = isometry_.linear() * _rotation;
      }
    }

    //! @breif operators
    //! ------------------------------------------------------------------- !//
    inline Matchable_<Scalar>& operator=(const Matchable_<Scalar>& other_) {
      _type              = other_._type;
      _origin            = other_._origin;
      _rotation          = other_._rotation;
      _activation_matrix = other_._activation_matrix;
      return *this;
    }

    inline Matchable_<Scalar>& operator+=(const Matchable_<Scalar>& other_) {
      if (_type != other_._type)
        throw std::runtime_error("Matchable_::operator+=|type mismatch");

      _origin.noalias() += other_._origin;

      // ia we have to normalize the final direction
      if (_type != Type::Point) {
        VectorType sum_direction = direction() + other_.direction();
        sum_direction.normalize();
        setDirection(sum_direction);
      }
      return *this;
    }

    inline Matchable_<Scalar> operator+(const Matchable_<Scalar>& other_) const {
      if (_type != other_._type)
        throw std::runtime_error("Matchable_::operator+|type mismatch");
      Matchable_<Scalar> sum(*this);
      sum += other_;
      return sum;
    }

    inline Matchable_<Scalar>& operator-=(const Matchable_<Scalar>& other_) {
      if (_type != other_._type)
        throw std::runtime_error("Matchable_::operator+|type mismatch");

      _origin.noalias() -= other_._origin;

      // ia we have to normalize the final direction
      if (_type != Type::Point) {
        VectorType diff_direction = direction() - other_.direction();
        diff_direction.normalize();
        setDirection(diff_direction);
      }
      return *this;
    }

    inline Matchable_<Scalar> operator-(const Matchable_<Scalar>& other_) const {
      if (_type != other_._type)
        throw std::runtime_error("Matchable_::operator+|type mismatch");
      Matchable_<Scalar> sum(*this);
      sum -= other_;
      return sum;
    }

    inline Matchable_<Scalar>& operator*=(const Scalar& s_) {
      _origin *= s_;
      return *this;
    }

    inline Matchable_<Scalar> operator*(const Scalar& s_) const {
      Matchable_<Scalar> dest(*this);
      dest *= s_;
      return dest;
    }
    //! ------------------------------------------------------------------- !//

    //! @brief static functions
    //! ------------------------------------------------------------------- !//
    static inline Matchable_<Scalar> Identity() {
      Matchable_<Scalar> returned;
      returned.origin().setZero();
      returned.rotation().setIdentity();
      return returned;
    }

    static inline Matchable_<Scalar> Zero() {
      Matchable_<Scalar> returned;
      returned.origin().setZero();
      returned.rotation().setIdentity();
      return returned;
    }
    //! ------------------------------------------------------------------- !//

    //! @brief resets the omega (used in the activation matrix)
    //!        on the basis of the type of the matchable
    void resetOmegaFromType();

    //! @brief sets the origin to 0 and the _rotation to identity
    //!        type, remains unchanged
    inline void setZero() {
      _origin.setZero();
      _rotation.setIdentity();
    }

  public:
    //! @brief origin accessors
    inline const VectorType& origin() const {
      return _origin;
    }

    inline VectorType& origin() {
      return _origin;
    }

    //! @brief rotation accessors
    inline const MatrixType& rotation() const {
      return _rotation;
    }

    inline MatrixType& rotation() {
      return _rotation;
    }

    //! @brief origin accessors
    inline const MatrixType& activation() const {
      return _activation_matrix;
    }

    inline MatrixType& activation() {
      return _activation_matrix;
    }

  protected:
    //! @brief _origin xyz
    VectorType _origin;
    //! @brief rotation matrix that describes normal direction
    MatrixType _rotation;
    //! @brief complementary matrix that shapes the error function
    //!        based on the matchable type
    MatrixType _activation_matrix;

    //! @brief avoids rank losses of matrix _activation_matrix
    static constexpr Scalar _epsilon = 1e-6;

  public:
    //! @brief overload of << operator
    // friend std::ostream& operator<<(std::ostream& stream_,
    //                                 const Matchable_<Scalar_>& matchable_) {
    //   stream_ << "[ type = " << matchable_.type << "\t"
    //           << "orig = " << matchable_._origin.transpose() << "\t"
    //           << "dir  = " << matchable_.direction().transpose() << " ]";
    //   return stream_;
    // }
  };

  //! @brief scalar usings
  using Matchabled = Matchable_<double>;
  using Matchablef = Matchable_<float>;

  //! @brief basic containers
  template <typename Scalar_>
  using MatchableVector_ =
    std::vector<Matchable_<Scalar_>, Eigen::aligned_allocator<Matchable_<Scalar_>>>;
  using MatchablefVector = MatchableVector_<float>;
  using MatchabledVector = MatchableVector_<double>;

} // namespace srrg2_core

#include "matchable.hpp"
