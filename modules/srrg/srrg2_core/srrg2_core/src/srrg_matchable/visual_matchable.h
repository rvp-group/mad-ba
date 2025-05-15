#pragma once
#include "srrg_matchable/descriptor_owner.h"
#include "srrg_matchable/matchable.h"
#include "srrg_pcl/point_normal_curvature_color.h"

namespace srrg2_core {

  //! @brief framepoint of the scene with attached descriptor
  //!        [TODO] name sucks
  template <typename Scalar_>
  class VisualMatchable_ : public Matchable_<Scalar_>, public DescriptorOwnerMatBinary {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief usings
    using Scalar = Scalar_;

    using ThisType            = VisualMatchable_<Scalar_>;
    using MatchableType       = Matchable_<Scalar_>;
    using DescriptorOwnerType = DescriptorOwnerMatBinary;
    using ExtentType          = PointNormalCurvatureColor3f;
    using ExtentCloudType     = PointNormalCurvatureColor3fVectorCloud;

    using VectorType         = Vector3_<Scalar>;
    using MatrixType         = Matrix3_<Scalar>;
    static constexpr int Dim = VectorType::RowsAtCompileTime + // ia origin
                               MatrixType::RowsAtCompileTime + // ia direction
                               1;                              // ia type
    using FullVectorType = Vector_<Scalar, Dim>;

    VisualMatchable_() : MatchableType(), DescriptorOwnerType() {
    }

    //! @brief ctor from origin and rotation matrix
    VisualMatchable_(const MatchableBase::Type& type_,
                     const VectorType& origin_   = VectorType::Zero(),
                     const MatrixType& rotation_ = MatrixType::Identity()) :
      MatchableType(type_, origin_, rotation_),
      DescriptorOwnerType() {
      _color_rgb.setZero();
    }

    //! @brief ctor from vectorized matchable
    VisualMatchable_(const FullVectorType& vector_) :
      MatchableType(vector_),
      DescriptorOwnerType() {
      _color_rgb.setZero();
      // todo color_rgb to set
    }

    //! @brief copy ctor
    VisualMatchable_(const ThisType& other_) :
      MatchableType(other_._type, other_._origin, other_._rotation),
      DescriptorOwnerType(other_._descriptor) {
      if (other_._support_cloud.size()) {
        _support_cloud = other_._support_cloud;
      }
      _color_rgb = other_._color_rgb;
    }

    //! @brief specialization of functions derived from matchable,
    //!        to change the return type

    //! @brief apply a transform to the matchable
    inline ThisType transform(const Isometry3_<Scalar>& isometry_) const {
      ThisType dest(*this);
      dest.transformInPlace(isometry_);
      return dest;
    }

    //! @brief trasforms the matchable and the support in place
    inline void transformSupportInPlace(const Isometry3_<Scalar>& isometry_) {
      this->transformInPlace(isometry_);
      this->support().transformInPlace(isometry_);
    }

    //! @brief trasforms the matchable and the support in a returned matchable
    inline ThisType transformSupport(const Isometry3_<Scalar>& isometry_) const {
      ThisType dest(*this);
      dest.transformSupportInPlace(isometry_);
      return dest;
    }

    //! -----------------------------------------------------------------------
    //! @breif operators [rollback to matchable ones]
    inline ThisType& operator=(const ThisType& other_) {
      MatchableType::operator=(other_);
      _color_rgb             = other_._color_rgb;
      if (other_._support_cloud.size()) {
        _support_cloud = other_._support_cloud;
      }
      other_._descriptor.copyTo(this->_descriptor);
      return *this;
    }

    inline ThisType& operator+=(const ThisType& other_) {
      MatchableType::operator+=(other_);
      return *this;
    }

    inline ThisType operator+(const ThisType& other_) const {
      ThisType sum(*this);
      sum += other_;
      return sum;
    }

    inline ThisType& operator-=(const ThisType& other_) {
      MatchableType::operator-=(other_);
      return *this;
    }

    inline ThisType operator-(const ThisType& other_) const {
      ThisType sum(*this);
      sum -= other_;
      return sum;
    }

    inline ThisType& operator*=(const Scalar& s_) {
      this->_origin *= s_;
      return *this;
    }

    inline ThisType operator*(const Scalar& s_) const {
      ThisType dest(*this);
      dest *= s_;
      return dest;
    }
    //! -----------------------------------------------------------------------

  public:
    //! @brief accessors
    inline const Vector3f& color() const {
      return _color_rgb;
    }
    inline Vector3f& color() {
      return _color_rgb;
    }

    inline const PointNormalCurvatureColor3fVectorCloud& support() const {
      return _support_cloud;
    }
    inline PointNormalCurvatureColor3fVectorCloud& support() {
      return _support_cloud;
    }

  protected:
    //! @brief color to be rendered
    Vector3f _color_rgb;

    //! @brief cluster cloud (already transformed and voxelized)
    //!        just to correctly visualize planes
    PointNormalCurvatureColor3fVectorCloud _support_cloud;
  };

  //! @brief scalar type
  using VisualMatchablef = VisualMatchable_<float>;
  using VisualMatchabled = VisualMatchable_<double>;

  //! @brief containers
  template <typename Scalar_>
  using VisualMatchableVector_ =
    std::vector<VisualMatchable_<Scalar_>, Eigen::aligned_allocator<VisualMatchable_<Scalar_>>>;

  using VisualMatchablefVector = VisualMatchableVector_<float>;
  using VisualMatchabledVector = VisualMatchableVector_<double>;

} /* namespace srrg2_core */
