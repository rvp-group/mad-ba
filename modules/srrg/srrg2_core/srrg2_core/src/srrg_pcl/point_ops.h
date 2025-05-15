#pragma once
#include <iostream>

namespace srrg2_core {

  // variadic recursive implementation of point operations
  // two classes, one for the inductive step
  // one for the base step
  // it relies on well formed traits for the points

  template <typename PointType_, int i = PointType_::NumFields - 1>
  struct PointOps_ {
    static constexpr int _Dimensions =
      PointType_::template TypeAt<i>::Dim + PointOps_<PointType_, i - 1>::_Dimensions;

    template <typename ScalarPtr_>
    inline static void toPlainVector(ScalarPtr_ dest, const PointType_& src) {
      PointOps_<PointType_, i - 1>::toPlainVector(dest, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      dest += PointOps_<PointType_, i - 1>::_Dimensions;
      FieldTraits::copyTo(dest, src.template value<i>());
    }

    template <typename ScalarPtr_>
    inline static void fromPlainVector(PointType_& dest, ScalarPtr_ src) {
      PointOps_<PointType_, i - 1>::fromPlainVector(dest, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      src += PointOps_<PointType_, i - 1>::_Dimensions;
      FieldTraits::copyFrom(dest.template value<i>(), src);
    }

    inline static void setZero(PointType_& dest) {
      PointOps_<PointType_, i - 1>::setZero(dest);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::setZero(dest.template value<i>());
    }

    inline static void normalize(PointType_& dest) {
      PointOps_<PointType_, i - 1>::normalize(dest);
      if (dest.status != Valid)
        return;
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      dest.status       = FieldTraits::normalize(dest.template value<i>());
    }

    inline static std::ostream& toStream(std::ostream& os, const PointType_& src) {
      PointOps_<PointType_, i - 1>::toStream(os, src);
      os << ", ";
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::toStream(os, src.template value<i>());
      return os;
    }

    template <typename Scalar_>
    inline static void scaleInPlace(PointType_& dest, const Scalar_& s) {
      PointOps_<PointType_, i - 1>::scaleInPlace(dest, s);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::scaleInPlace(dest.template value<i>(), s);
    }

    template <typename Scalar_>
    inline static void scale(PointType_& dest, const Scalar_& s, const PointType_& src) {
      PointOps_<PointType_, i - 1>::scale(dest, s, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::scale(dest.template value<i>(), s, src.template value<i>());
    }

    inline static void addInPlace(PointType_& dest, const PointType_& src) {
      PointOps_<PointType_, i - 1>::addInPlace(dest, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::addInPlace(dest.template value<i>(), src.template value<i>());
    }

    inline static void add(PointType_& dest, const PointType_& src1, const PointType_& src2) {
      PointOps_<PointType_, i - 1>::add(dest, src1, src2);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::add(
        dest.template value<i>(), src1.template value<i>(), src2.template value<i>());
    }

    template <typename Scalar_>
    inline static void addAndScaleInPlace(PointType_& dest, Scalar_ s, const PointType_& src) {
      PointOps_<PointType_, i - 1>::addAndScaleInPlace(dest, s, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::addAndScaleInPlace(dest.template value<i>(), s, src.template value<i>());
    }

    template <TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry, typename TransformType_>
    inline static void transformInPlace(PointType_& dest, const TransformType_& transform) {
      PointOps_<PointType_, i - 1>::template transformInPlace<transform_class, TransformType_>(
        dest, transform);
      if (dest.status != POINT_STATUS::Valid)
        return;
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      dest.status       = FieldTraits::template transformInPlace<TransformType_, transform_class>(
        dest.template value<i>(), transform);
    }

    template <TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry, typename TransformType_>
    inline static void
    transform(PointType_& dest, const TransformType_& transform, const PointType_& src) {
      PointOps_<PointType_, i - 1>::template transform<transform_class, TransformType_>(
        dest, transform, src);
      if (dest.status != POINT_STATUS::Valid)
        return;

      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      dest.status       = FieldTraits::template transform<TransformType_, transform_class>(
        dest.template value<i>(), transform, src.template value<i>());
    }

    inline static void euclidean2polar(PointType_& dest, const PointType_& src) {
      PointOps_<PointType_, i - 1>::euclidean2polar(dest, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::euclidean2polar(dest.template value<i>(), src.template value<i>());
    }

    inline static void polar2euclidean(PointType_& dest, const PointType_& src) {
      PointOps_<PointType_, i - 1>::polar2euclidean(dest, src);
      using FieldTraits = typename PointType_::template TraitsTypeAt<i>;
      FieldTraits::polar2euclidean(dest.template value<i>(), src.template value<i>());
    }

    // ds copies all overlapping fields from src to dest
    // ds will not compile if dest has not a subset/conflics with src's fields
    template <typename PointTypeSource_>
    inline static void copyFields(PointType_& dest, const PointTypeSource_& src) {
      PointOps_<PointType_, i - 1>::copyFields(dest, src);
      dest.template field<i>() = src.template field<i>();
    }
  };

  template <typename PointType_>
  struct PointOps_<PointType_, 0> {
    static constexpr int _Dimensions = PointType_::template TypeAt<0>::Dim;

    template <typename ScalarPtr_>
    inline static void toPlainVector(ScalarPtr_ dest, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::copyTo(dest, src.template value<0>());
    }

    template <typename ScalarPtr_>
    inline static void fromPlainVector(PointType_& dest, ScalarPtr_ src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::copyFrom(dest.template value<0>(), src);
    }

    inline static std::ostream& toStream(std::ostream& os, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::toStream(os, src.template value<0>());
      return os;
    }

    template <typename Scalar_>
    inline static void scaleInPlace(PointType_& dest, const Scalar_& s) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::scaleInPlace(dest.template value<0>(), s);
    }

    template <typename Scalar_>
    inline static void scale(PointType_& dest, const Scalar_& s, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::scale(dest.template value<0>(), s, src.template value<0>());
    }

    inline static void addInPlace(PointType_& dest, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::addInPlace(dest.template value<0>(), src.template value<0>());
    }

    inline static void add(PointType_& dest, const PointType_& src1, const PointType_& src2) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::add(
        dest.template value<0>(), src1.template value<0>(), src2.template value<0>());
    }

    template <typename Scalar_>
    inline static void addAndScaleInPlace(PointType_& dest, Scalar_ s, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::addAndScaleInPlace(dest.template value<0>(), s, src.template value<0>());
    }

    template <TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry, typename TransformType_>
    inline static void transformInPlace(PointType_& dest, const TransformType_& transform) {
      if (dest.status != POINT_STATUS::Valid)
        return;
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      dest.status       = FieldTraits::template transformInPlace<TransformType_, transform_class>(
        dest.template value<0>(), transform);
    }

    template <TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry, typename TransformType_>
    inline static void
    transform(PointType_& dest, const TransformType_& transform, const PointType_& src) {
      if (src.status != POINT_STATUS::Valid)
        return;
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      dest.status       = FieldTraits::template transform<TransformType_, transform_class>(
        dest.template value<0>(), transform, src.template value<0>());
    }

    inline static void euclidean2polar(PointType_& dest, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::euclidean2polar(dest.template value<0>(), src.template value<0>());
    }

    inline static void polar2euclidean(PointType_& dest, const PointType_& src) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::polar2euclidean(dest.template value<0>(), src.template value<0>());
    }

    inline static void setZero(PointType_& dest) {
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      FieldTraits::setZero(dest.template value<0>());
    }

    inline static void normalize(PointType_& dest) {
      if (dest.status != Valid)
        return;
      using FieldTraits = typename PointType_::template TraitsTypeAt<0>;
      dest.status       = FieldTraits::normalize(dest.template value<0>());
    }

    // ds recursion termination specialization
    // ds copies all overlapping fields from src to dest
    // ds will not compile if dest has not a subset/conflics with src's fields
    template <typename PointTypeSource_>
    inline static void copyFields(PointType_& dest, const PointTypeSource_& src) {
      dest.template field<0>() = src.template field<0>();
    }
  };

} // namespace srrg2_core
