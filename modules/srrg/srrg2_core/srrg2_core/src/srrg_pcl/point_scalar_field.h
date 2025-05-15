#pragma once
#include "point_default_field.h"
#include "point_defs.h"

namespace srrg2_core {

  //! traits that define the operations on a field of a point
  template <typename PointFieldType_>
  struct PointScalarFieldTraits_
    : public PointDefaultFieldTraits_<PointFieldType_> {
    using Scalar = float;

    using PointFieldType = PointFieldType_;

    //! Dimension of the field, in case of scalar it is one
    static constexpr int Dim = 1;

    // copy to a n array of Scalars
    template <typename ScalarPtr>
    static inline void copyTo(ScalarPtr dest, const PointFieldType& src) {
      *dest = src;
    }

    // copy from an array of scalars
    template <typename ScalarPtr>
    static inline void copyFrom(PointFieldType& dest, const ScalarPtr src) {
      dest = *src;
    }

    // +=  operator,
    static inline void addInPlace(PointFieldType& dest,
                                  const PointFieldType& src) {
      dest += src;
    }

    // +  operator
    static inline void add(PointFieldType& dest,
                           const PointFieldType& src1,
                           const PointFieldType& src2) {
      dest = src1 + src2;
    }

    // dest += s*src
    static inline void addAndScaleInPlace(PointFieldType& dest,
                                          const Scalar& s,
                                          const PointFieldType& src) {
      dest += s * src;
    }

    static inline void
    scale(PointFieldType& dest, const Scalar& s, const PointFieldType& src) {
      dest = s * src;
    }

    // dest *= s
    static inline void scaleInPlace(PointFieldType& dest, const Scalar& s) {
      dest *= s;
    }

    // other functions inherited from the base trait
    // dest = transform(src), returns if the point is ok
    /*
    template <typename TransformType_,
        enum TRANSFORM_CLASS transform_class=TRANSFORM_CLASS::Isometry>
    static inline POINT_STATUS transform(PointFieldType& dest,
                                         const TransformType_& transform,
                                         const PointFieldType& src){
      dest=src;
      return POINT_STATUS::Valid;
    }

    // dest = transform(dest)
    template <typename TransformType_,
        enum TRANSFORM_CLASS transform_class=TRANSFORM_CLASS::Isometry>
    static inline POINT_STATUS transformInPlace(PointFieldType& dest,
                                                const TransformType_&
    transform){ return POINT_STATUS::Valid;
    }

    // dest=toPolar(src)
    static inline void euclidean2polar(PointFieldType& dest,
               const PointFieldType& src){
      dest=src;
    }

    // dest=fromPolar(src)
    static inline void polar2euclidean(PointFieldType& dest,
               const PointFieldType& src){
      dest=src;
    }
    */

    static inline void setZero(PointFieldType& src) {
      src = PointFieldType(0);
    }
  };

  template <typename ValueType_>
  struct PointScalarField_ {
    static constexpr int Dim = 1;
    using ValueType          = ValueType_;
    using TraitsType         = PointScalarFieldTraits_<ValueType_>;
    ValueType value;
  };

} // namespace srrg2_core
