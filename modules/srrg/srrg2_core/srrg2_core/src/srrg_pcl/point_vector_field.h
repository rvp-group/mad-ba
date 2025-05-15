#pragma once
#include "point_defs.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {

  // traits for a field of vector type
  template <typename PointFieldType_,
            bool SupportsTransform_,  // true if item can be transformed
            bool SupportsPolar_,      // true if item can be polar converted
            bool SupportsProjection_, // true if item can supports polar
                                      // projection
            bool IsDirection_         // true if item is a unit vector
            >
  struct PointVectorFieldTraits_ {
    static constexpr bool SupportsTransform  = SupportsTransform_;
    static constexpr bool SupportsPolar      = SupportsPolar_;
    static constexpr bool SupportsProjection = SupportsProjection_;
    static constexpr bool IsDirection        = IsDirection_;

    using Scalar             = typename PointFieldType_::Scalar;
    using PointFieldType     = PointFieldType_;
    static constexpr int Dim = PointFieldType::RowsAtCompileTime;

    template <typename ScalarPtr>
    static inline void copyTo(ScalarPtr dest, const PointFieldType& src) {
      for (int i = 0; i < Dim; ++i, ++dest)
        *dest = src(i);
    }

    template <typename ScalarPtr>
    static inline void copyFrom(PointFieldType& dest, ScalarPtr src) {
      for (int i = 0; i < Dim; ++i, ++src)
        dest(i) = *src;
    }

    static inline void addInPlace(PointFieldType& dest, const PointFieldType& src) {
      dest += src;
    }

    static inline void
    add(PointFieldType& dest, const PointFieldType& src1, const PointFieldType& src2) {
      dest = src1 + src2;
    }

    static inline void
    addAndScaleInPlace(PointFieldType& dest, const Scalar& s, const PointFieldType& src) {
      dest += src * s;
    }

    static inline void scale(PointFieldType& dest, const Scalar& s, const PointFieldType& src) {
      dest = src * s;
    }

    static inline void scaleInPlace(PointFieldType& dest, const Scalar& s) {
      dest *= s;
    }

    template <typename TransformType_,
              enum TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry>
    static inline POINT_STATUS
    transform(PointFieldType& dest, const TransformType_& transform, const PointFieldType& src) {
      if (!SupportsTransform) {
        dest = src;
        return POINT_STATUS::Valid;
      }
      static const Scalar one(1.0);
      switch (transform_class) {
        case TRANSFORM_CLASS::Isometry:
          if (IsDirection) {
            dest = transform.linear() * src;
            return POINT_STATUS::Valid;
          }
          dest = transform * src;
          return POINT_STATUS::Valid;
        case TRANSFORM_CLASS::PinholeProjection:
          if (IsDirection) {
            dest = src;
            return POINT_STATUS::Valid;
          }
          dest = transform * src;
          if (dest(Dim - 1) > 0) {
            const Scalar inv_z = one / dest(Dim - 1);
            dest.head(Dim - 1) *= inv_z;
            return POINT_STATUS::Valid;
          } else {
            return POINT_STATUS::BehindObserver;
          }
        case TRANSFORM_CLASS::PinholeUnprojection:
          dest = src;
          if (IsDirection) {
            return POINT_STATUS::Valid;
          }
          dest.head(Dim - 1) *= src(Dim - 1);
          dest = transform * dest;
          return POINT_STATUS::Valid;
          break;
        case TRANSFORM_CLASS::PolarProjection:
          if (IsDirection) {
            dest = src;
            return POINT_STATUS::Valid;
          }
          euclidean2polar(dest, src);
          dest = transform * dest;
          return POINT_STATUS::Valid;
        case TRANSFORM_CLASS::PolarUnprojection:
          if (IsDirection) {
            dest = src;
            return POINT_STATUS::Valid;
          }
          dest = transform * src;
          polar2euclidean(dest);
          return POINT_STATUS::Valid;
        default:;
      }
      return POINT_STATUS::Valid;
    }

    template <typename TransformType_,
              enum TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry>
    static inline POINT_STATUS transformInPlace(PointFieldType& dest,
                                                const TransformType_& transform) {
      if (!SupportsTransform) {
        return POINT_STATUS::Valid;
      }
      static const Scalar one(1.0);
      switch (transform_class) {
        case TRANSFORM_CLASS::Isometry:
          if (IsDirection) {
            dest = transform.linear() * dest;
            return POINT_STATUS::Valid;
          }
          dest = transform * dest;
          return POINT_STATUS::Valid;
        case TRANSFORM_CLASS::PinholeProjection:
          if (IsDirection) {
            return POINT_STATUS::Valid;
          }
          dest = transform * dest;
          if (dest(Dim - 1) > 0) {
            const Scalar inv_z = one / dest(Dim - 1);
            dest.head(Dim - 1) *= inv_z;
            return POINT_STATUS::Valid;
          } else {
            return POINT_STATUS::BehindObserver;
          }
          return POINT_STATUS::Valid;
        case TRANSFORM_CLASS::PinholeUnprojection:
          if (IsDirection) {
            return POINT_STATUS::Valid;
          }
          dest.head(Dim - 1) *= dest(Dim - 1);
          dest = transform * dest;
          return POINT_STATUS::Valid;
        case TRANSFORM_CLASS::PolarProjection:
          if (IsDirection) {
            return POINT_STATUS::Valid;
          }
          euclidean2polar(dest);
          dest = transform * dest;
          return POINT_STATUS::Valid;
        case TRANSFORM_CLASS::PolarUnprojection:
          if (IsDirection) {
            return POINT_STATUS::Valid;
          }
          dest = transform * dest;
          polar2euclidean(dest);
          return POINT_STATUS::Valid;
        default:;
      }
      return POINT_STATUS::Valid;
    }

    static inline void euclidean2polar(PointFieldType& dest, const PointFieldType& src) {
      if (!SupportsPolar) {
        dest = src;
        return;
      }

      if (Dim == 2) {
        dest(0) = atan2(src(1), src(0));
        dest(1) = src.norm();
        return;
      }
      if (Dim == 3) {
        dest(0) = atan2(src(1), src(0));
        dest(1) = atan2(src(2), src.head(2).norm());
        dest(2) = src.norm();
        return;
      }
    }

    static inline void polar2euclidean(PointFieldType& dest, const PointFieldType& src) {
      if (!SupportsPolar) {
        dest = src;
        return;
      }

      if (Dim == 2) {
        float s0 = sin(src(0));
        float c0 = cos(src(0));
        dest(0)  = c0 * src(1);
        dest(1)  = s0 * src(1);
        return;
      }
      if (Dim == 3) {
        float s0 = sin(src(0));
        float c0 = cos(src(0));
        float s1 = sin(src(1));
        float c1 = cos(src(1));
        dest(0)  = c0 * c1 * src(2);
        dest(1)  = s0 * c1 * src(2);
        dest(2)  = s1 * src(2);
        return;
      }
    }

    static inline void euclidean2polar(PointFieldType& dest) {
      PointFieldType aux;
      euclidean2polar(aux, dest);
      dest = aux;
    }

    static inline void polar2euclidean(PointFieldType& dest) {
      PointFieldType aux;
      polar2euclidean(aux, dest);
      dest = aux;
    }

    static inline POINT_STATUS normalize(PointFieldType& src) {
      if (!IsDirection) {
        return POINT_STATUS::Valid;
      }
      if (src.squaredNorm() == 0)
        return POINT_STATUS::Invalid;
      src.normalize();
      return POINT_STATUS::Valid;
    }

    static inline std::ostream& toStream(std::ostream& os, const PointFieldType& src) {
      os << "(" << src.transpose() << ")";
      return os;
    }

    static inline void setZero(PointFieldType& src) {
      src.setZero();
    }
  };

  // Fields:
  //  bool SupportsTransform_,  // true if item can be transformed
  //  bool SupportsPolar_,      // true if item can be polar converted
  //  bool SupportsProjection_, // true if item can supports polar projection
  //  bool IsDirection_         // true if item is a unit vector

  template <typename Scalar_, int Dim_>
  struct PointCoordinatesField_ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int Dim = Dim_;
    using ValueType          = Vector_<Scalar_, Dim_>;
    using TraitsType         = PointVectorFieldTraits_<ValueType, true, true, true, false>;
    ValueType value;
  };

  template <typename Scalar_, int Dim_>
  struct PointDirectionField_ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int Dim = Dim_;
    using ValueType          = Vector_<Scalar_, Dim_>;
    using TraitsType         = PointVectorFieldTraits_<ValueType, true, false, false, true>;
    ValueType value;
  };

  template <typename Scalar_, int Dim_>
  struct PointColorField_ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int Dim = Dim_;
    using ValueType          = Vector_<Scalar_, Dim_>;
    using TraitsType         = PointVectorFieldTraits_<ValueType, false, false, false, false>;
    ValueType value;
  };

} // namespace srrg2_core
