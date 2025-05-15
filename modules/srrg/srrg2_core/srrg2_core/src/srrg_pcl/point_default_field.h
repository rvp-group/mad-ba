#pragma once
#include <iostream>

#include "point_defs.h"

namespace srrg2_core {

  //! traits that define the operations on a field of a point
  template <typename PointFieldType_>
  struct PointDefaultFieldTraits_ {
    using Scalar = float;

    using PointFieldType = PointFieldType_;

    //! Dimension of a point, in case of a value element
    //! this is zero, in case of a value that supports arithmetic operations
    //! this is the number of elements
    static constexpr int Dim = 0;

    // copy to a n array of Scalars
    template <typename ScalarPtr>
    static inline void copyTo(ScalarPtr dest, const PointFieldType& src) {
    }

    // copy from an array of scalars
    template <typename ScalarPtr>
    static inline void copyFrom(PointFieldType& dest, const ScalarPtr src) {
    }

    // +=  operator
    static inline void addInPlace(PointFieldType& dest,
                                  const PointFieldType& src) {
    }

    // +  operator
    static inline void add(PointFieldType& dest,
                           const PointFieldType& src1,
                           const PointFieldType& src2) {
      dest = src1;
    }

    // dest += s*src
    static inline void addAndScaleInPlace(PointFieldType& dest,
                                          const Scalar& s,
                                          const PointFieldType& src) {
    }

    static inline void
    scale(PointFieldType& dest, const Scalar& s, const PointFieldType& src) {
      dest = src;
    }

    // dest *= s
    static inline void scaleInPlace(PointFieldType& dest, const Scalar& s) {
    }

    // dest = transform(src), returns if the point is ok
    template <typename TransformType_,
              enum TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry>
    static inline POINT_STATUS transform(PointFieldType& dest,
                                         const TransformType_& transform,
                                         const PointFieldType& src) {
      dest = src;
      return POINT_STATUS::Valid;
    }

    // dest = transform(dest)
    template <typename TransformType_,
              enum TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry>
    static inline POINT_STATUS
    transformInPlace(PointFieldType& dest, const TransformType_& transform) {
      return POINT_STATUS::Valid;
    }

    // dest=toPolar(src)
    static inline void euclidean2polar(PointFieldType& dest,
                                       const PointFieldType& src) {
      dest = src;
    }

    // dest=fromPolar(src)
    static inline void polar2euclidean(PointFieldType& dest,
                                       const PointFieldType& src) {
      dest = src;
    }

    // dest=toPolar(src)
    static inline void euclidean2polar(PointFieldType& dest) {
    }

    // dest=fromPolar(src)
    static inline void polar2euclidean(PointFieldType& dest) {
    }

    static inline std::ostream& toStream(std::ostream& os,
                                         const PointFieldType& src) {
      os << "(" << src << ")";
      return os;
    }

    static inline POINT_STATUS normalize(PointFieldType& src) {
      return POINT_STATUS::Valid;
    }

    static inline void setZero(PointFieldType& src) {
    }
  };

  template <typename ValueType_>
  struct PointDefaultField_ {
    static constexpr int Dim = 0;
    using ValueType          = ValueType_;
    using TraitsType         = PointDefaultFieldTraits_<ValueType_>;
    ValueType value;
  };

} // namespace srrg2_core
