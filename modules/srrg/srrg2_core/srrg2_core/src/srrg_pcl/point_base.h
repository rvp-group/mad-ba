#pragma once
#include "srrg_data_structures/field_pack.h"

#include "point_fields.h"
#include "point_ops.h"

namespace srrg2_core {

  template <int GeometryDim_, typename... PointFieldsType_>
  struct PointBase_ : public FieldPack_<PointFieldsType_...> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = FieldPack_<PointFieldsType_...>;
    using ThisType = PointBase_<GeometryDim_, PointFieldsType_...>;

    // core dimension of the point (2D, 3D...);
    static constexpr int GeometryDim = GeometryDim_;

    // cumulative dimension of all data fields
    static constexpr int Dimensions = PointOps_<ThisType, BaseType::NumFields - 1>::_Dimensions;

    // coefficient vector, it only stores the algebric elements
    template <typename Scalar_>
    using PlainVectorType = Vector_<Scalar_, Dimensions>;

    // type of the field at i
    template <int idx>
    using ValueTypeAt = typename BaseType::template TypeAt<idx>::ValueType; 

    // type of the traits at i
    template <int idx>
    using TraitsTypeAt = typename BaseType::template TypeAt<idx>::TraitsType;

    // position of the ith field in the plain vector
    template <int idx>
    static constexpr int FieldOffset =
      PointOps_<ThisType, idx>::_Dimensions - BaseType::template TypeAt<idx>::Dim;

    // value of the ith field
    template <int idx>
    inline ValueTypeAt<idx>& value() {
      return BaseType::template field<idx>().value;
    }

    // same as above, but const
    template <int idx>
    inline const ValueTypeAt<idx>& value() const {
      return BaseType::template field<idx>().value;
    }

    // converts to an Eigen vector
    template <typename Scalar_>
    inline void toPlainVector(Vector_<Scalar_, Dimensions>& dest) const {
      PointOps_<ThisType, BaseType::NumFields - 1>::toPlainVector(&dest(0), *this);
    }

    // converts from an Eigen vector
    template <typename Scalar_>
    inline void fromPlainVector(const Vector_<Scalar_, Dimensions>& src) {
      PointOps_<ThisType, BaseType::NumFields - 1>::fromPlainVector(*this, &src(0));
    }

    inline ThisType& normalize() {
      PointOps_<ThisType, BaseType::NumFields - 1>::normalize(*this);
      return *this;
    }

    template <int idx>
    inline ThisType& normalize() {
      using Traits = TraitsTypeAt<idx>;
      Traits::normalize(value<idx>());
      return *this;
    }

    // ia set every field to zero
    inline ThisType& setZero() {
      PointOps_<ThisType, BaseType::NumFields - 1>::setZero(*this);
      return *this;
    }

    template <int idx>
    inline ThisType& setZero() {
      using Traits = TraitsTypeAt<idx>;
      Traits::setZero(value<idx>());
      return *this;
    }

    // operations
    template <typename Scalar_>
    inline ThisType& operator*=(const Scalar_& s) {
      PointOps_<ThisType, BaseType::NumFields - 1>::scaleInPlace(*this, s);
      return *this;
    }

    template <int idx, typename Scalar_>
    inline ThisType& operator*=(const Scalar_& s) {
      using Traits = typename ThisType::template TraitsTypeAt<idx>;
      Traits::scaleInPlace(value<idx>(), s);
      return *this;
    }

    template <typename Scalar_>
    inline ThisType operator*(const Scalar_& s) const {
      ThisType returned;
      PointOps_<ThisType, BaseType::NumFields - 1>::scale(returned, s, *this);
      return returned;
    }

    template <int idx, typename Scalar_>
    inline ThisType operator*(const Scalar_& s) const {
      ThisType returned = *this;
      using Traits      = typename ThisType::template TraitsTypeAt<idx>;
      Traits::scale(returned.template value<idx>(), s, ThisType::template value<idx>());
      return returned;
    }

    //! @brief point with point addition
    inline ThisType& operator+=(const ThisType& other) {
      PointOps_<ThisType, BaseType::NumFields - 1>::addInPlace(*this, other);
      return *this;
    }

    template <int idx>
    inline ThisType& operator+=(const ThisType& other) {
      using Traits = typename ThisType::template TraitsTypeAt<idx>;
      Traits::addInPlace(value<idx>(), other.value<idx>());
      return *this;
    }

    inline ThisType operator+(const ThisType& other) const {
      ThisType returned;
      PointOps_<ThisType, BaseType::NumFields - 1>::add(returned, *this, other);
      return returned;
    }

    template <int idx>
    inline ThisType operator+(const ThisType& other) const {
      ThisType returned = *this;
      using Traits      = typename ThisType::template TraitsTypeAt<idx>;
      Traits::add(returned.value<idx>(), returned.value<idx>(), other.value<idx>());
      return returned;
    }

    //! @brief subtraction between points
    inline ThisType& operator-=(const ThisType& other) {
      PointOps_<ThisType, BaseType::NumFields - 1>::addInPlace(*this, other * -1.0);
      return *this;
    }

    template <int idx>
    inline ThisType& operator-=(const ThisType& other) {
      using Traits = typename ThisType::template TraitsTypeAt<idx>;
      Traits::addInPlace(value<idx>(), other.value<idx>() * -1.0);
      return *this;
    }

    inline ThisType operator-(const ThisType& other) const {
      ThisType returned;
      PointOps_<ThisType, BaseType::NumFields - 1>::add(returned, *this, other * -1.0);
      return returned;
    }

    template <int idx>
    inline ThisType operator-(const ThisType& other) const {
      ThisType returned = *this;
      using Traits      = typename ThisType::template TraitsTypeAt<idx>;
      Traits::add(returned.value<idx>(), this->value<idx>(), other.value<idx>() * -1.0);
      return returned;
    }

    //! if transform_class= Isometry,            T: (R | t)
    //! if transform_class= PinholeProjection,   T: (K | 0)
    //! if transform_class= PinholeUnprojection, T: (K^{-1} | 0)
    //! if transform_class= PinholeProjection,   T: (K.block<2,2>(0,0) | K.block<2,1>(0,2))
    //! if transform_class= PinholeUnprojection, T: (K.block<2,2>(0,0)^{-1} 
    //!                                              | - K.block<2,2>(0,0)^{-1} * K.block<2,2>(0,0)^{-1})
    template <TRANSFORM_CLASS transform_class, typename TransformType_>
    inline ThisType transformInPlace(const TransformType_& transform) {
      PointOps_<ThisType, BaseType::NumFields - 1>::template transformInPlace<transform_class,
                                                                              TransformType_>(
        *this, transform);
      return *this;
    }

    template <int idx, TRANSFORM_CLASS transform_class, typename TransformType_>
    inline ThisType transformInPlace(const TransformType_& transform) {
      using Traits = typename ThisType::template TraitsTypeAt<idx>;
      Traits::template transformInPlace<TransformType_, transform_class>(
        ThisType::template value<idx>(), transform);
      return *this;
    }

    //! if transform_class= Isometry,            T: (R | t)
    //! if transform_class= PinholeProjection,   T: (KR | Kt)
    //! if transform_class= PinholeUnprojection, T: (R^T K^{-1} | - R^T t)
    template <TRANSFORM_CLASS transform_class, typename TransformType_>
    inline ThisType transform(const TransformType_& transform) const {
      ThisType returned;
      PointOps_<ThisType, BaseType::NumFields - 1>::template transform<transform_class,
                                                                       TransformType_>(
        returned, transform, *this);
      return returned;
    }

    template <int idx, TRANSFORM_CLASS transform_class, typename TransformType_>
    inline ThisType transform(const TransformType_& transform) const {
      ThisType returned = *this;
      using Traits      = TraitsTypeAt<idx>;
      Traits::template transform<TransformType_, transform_class>(
        returned.value<idx>(), transform, value<idx>());
      return returned;
    }

    inline ThisType euclidean2polar() {
      ThisType returned;
      PointOps_<ThisType, BaseType::NumFields - 1>::euclidean2polar(returned, *this);
      return returned;
    }

    template <int idx>
    inline ThisType euclidean2polar() {
      ThisType returned = *this;
      using Traits      = TraitsTypeAt<idx>;
      Traits::euclidean2polar(returned.value<idx>(), value<idx>());
      return returned;
    }

    inline ThisType polar2euclidean() {
      ThisType returned;
      PointOps_<ThisType, BaseType::NumFields - 1>::polar2euclidean(returned, *this);
      return returned;
    }

    template <int idx>
    inline ThisType polar2euclidean() {
      ThisType returned = *this;
      using Traits      = TraitsTypeAt<idx>;
      Traits::polar2euclidean(returned.value<idx>(), value<idx>());
      return returned;
    }

    std::ostream& toStream(std::ostream& os) const {
      os << "[";
      PointOps_<ThisType, BaseType::NumFields - 1>::toStream(os, *this);
      os << "]";
      return os;
    }

    // ds copies all overlapping fields from src to dest
    // ds will not compile if dest has not a subset/conflics with src's fields
    template <typename PointTypeSource_>
    inline void copyFields(const PointTypeSource_& src) {
      PointOps_<ThisType, BaseType::NumFields - 1>::template copyFields<PointTypeSource_>(*this,
                                                                                          src);
    }

    POINT_STATUS status = POINT_STATUS::Valid;
  };

} // namespace srrg2_core
