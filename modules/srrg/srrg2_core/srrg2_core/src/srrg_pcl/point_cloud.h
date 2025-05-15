#pragma once
#include "point_base.h"
#include "srrg_data_structures/matrix.h"
#include <vector>

namespace srrg2_core {
  template <typename ContainerType_>
  struct PointCloud_ : public ContainerType_ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType        = PointCloud_<ContainerType_>;
    using ContainerType   = ContainerType_;
    using PointType       = typename ContainerType_::value_type;
    using PlainVectorType = typename PointType::template PlainVectorType<float>;
    static constexpr int GeometryDim = PointType::GeometryDim;

    ThisType& setZero();

    // normalizes the idx field
    template <int idx>
    ThisType& setZero();

    // copies the src_idx_ field of this to the dest_idx field
    // of a destination cloud
    template <typename DestIteratorType_>
    void copyTo(DestIteratorType_& dest_it,
                const bool& keep_invalid_ = false) const {
      for (auto src_it = this->begin(); src_it != this->end(); ++src_it) {
        if (!keep_invalid_ && !(src_it->status == Valid)) {
          continue;
        }
        *dest_it++ = *src_it;
      }
    }

    // copies the src_idx_ field of this to the dest_idx field
    // of a destination cloud
    template <int dest_idx, int src_idx, typename DestCloudType_>
    void copyFieldTo(DestCloudType_& dest) const {
      assert(dest.size() == this->size());
      auto dest_it = dest.begin();
      for (auto src_it = this->begin(); src_it != this->end();
           ++src_it, ++dest_it) {
        dest_it->template value<dest_idx>() = src_it->template value<src_idx>();
      }
    }

    // copies the src_idx_ field of this to the dest_idx field
    // of a destination container
    template <int src_idx, typename DestRawType_>
    void copyRawFieldTo(DestRawType_& dest) const {
      // ia this is a shit, because if the point cloud is a vector you must use
      // size ia if it is a matrix you must use capacity
      assert(dest.size() == this->size());
      auto dest_it = dest.begin();
      for (auto src_it = this->begin(); src_it != this->end();
           ++src_it, ++dest_it) {
        *dest_it = src_it->template value<src_idx>();
      }
    }

    // copies the elements of a destination container to
    // the dest_idx field of this
    template <int dest_idx, typename SrcCloudType_>
    void copyRawFieldFrom(const SrcCloudType_& src) {
      assert(src.size() == this->size());
      auto dest_it = this->begin();
      for (auto src_it = src.begin(); src_it != src.end();
           ++src_it, ++dest_it) {
        dest_it->template value<dest_idx>() = *src_it;
      }
    }

    ThisType& normalize();

    // normalizes the idx field
    template <int idx>
    ThisType& normalize();

    // transforms the points in place
    // illegal points are marked as not valid
    // invalid points are ignored
    template <TRANSFORM_CLASS transform_class = TRANSFORM_CLASS::Isometry,
              typename TransformType_>
    ThisType& transformInPlace(const TransformType_& transform_);

    // transforms the idx field of the points in place
    // illegal points are marked as not valid
    // invalid points are ignored
    template <int idx, TRANSFORM_CLASS transform_class, typename TransformType_>
    ThisType& transformInPlace(const TransformType_& transform_);

    // transforms the points and puts in the target container
    // through an iterator
    // if a point is marked as invalid it is either dropped or retained
    // depending on suppress_invalids
    template <TRANSFORM_CLASS transform_class,
              typename OutputIteratorType_,
              typename TransformType_,
              bool suppress_invalids = true>
    void transform(OutputIteratorType_ target, const TransformType_&) const;

    // transforms the points and returns the resulting container
    template <TRANSFORM_CLASS transform_class, typename TransformType_>
    ThisType transform(const TransformType_& transform_) const;

    // same as above, but operates only on the idx field
    template <int idx,
              TRANSFORM_CLASS transform_class,
              typename OutputIteratorType_,
              typename TransformType_,
              bool suppress_invalids = true>
    void transform(OutputIteratorType_ target, const TransformType_&) const;

    // transforms the points and emplaces in the target container
    // the target should have the same size of the source, and is assumed
    // to be organized. Also invalid points are copied to keep the
    // bookkeeping
    template <TRANSFORM_CLASS transform_class, typename TransformType_>
    void transform(Matrix_<PointType>& dest,
                   const TransformType_& trans) const {
      assert(this->size() == dest->size());
      transform<transform_class,
                typename Matrix_<PointType>::iterator,
                TransformType_,
                false>(dest.begin(), trans);
    }

    // same as above, but operates only on the idx field
    template <int idx, TRANSFORM_CLASS transform_class, typename TransformType_>
    void transform(Matrix_<PointType>& dest,
                   const TransformType_& trans) const {
      assert(this->size() == dest->size());
      transform<idx,
                transform_class,
                typename Matrix_<PointType>::iterator,
                TransformType_,
                false>(dest.begin(), trans);
    }

    template <typename OutputIteratorType_>
    void voxelize(OutputIteratorType_ target, const PlainVectorType& scales);

    template <typename OutputIteratorType_>
    void clip(OutputIteratorType_ target,
              const PointType& origin,
              const PlainVectorType& scales);

  protected:
    static inline void invertScales(PlainVectorType& dest,
                                    const PlainVectorType& src) {
      for (int i = 0; i < PointType::Dimensions; ++i) {
        dest[i] = 0.f;
        if (src[i] > 0) {
          dest[i] = 1. / src[i];
        }
      }
    }

    using CellIndexArray = int[PointType::Dimensions];

    struct VoxelEntry {
      inline bool operator<(const VoxelEntry& other) const {
        for (int d = 0; d < PointType::Dimensions; ++d) {
          if (key[d] < other.key[d]) {
            return true;
          }
          if (key[d] > other.key[d]) {
            return false;
          }
        }
        return false;
      }
      inline void set(const typename ContainerType_::const_iterator& it_,
                      const PlainVectorType& inv_scales) {
        it = it_;
        PlainVectorType plain;
        it->toPlainVector(plain);
        for (int i = 0; i < PointType::Dimensions; ++i) {
          key[i] = int(inv_scales(i) * plain(i));
        }
      }
      CellIndexArray key;

      typename ContainerType_::const_iterator it;
    };
  };

  template <typename ContainerType_>
  PointCloud_<ContainerType_>& PointCloud_<ContainerType_>::setZero() {
    for (PointType& p : *this) {
      p.status = POINT_STATUS::Valid;
      p.setZero();
    }
    return *this;
  }

  template <typename ContainerType_>
  template <int idx>
  PointCloud_<ContainerType_>& PointCloud_<ContainerType_>::setZero() {
    for (PointType& p : *this) {
      p.status = POINT_STATUS::Valid;
      p.template setZero<idx>();
    }
    return *this;
  }

  template <typename ContainerType_>
  PointCloud_<ContainerType_>& PointCloud_<ContainerType_>::normalize() {
    for (PointType& p : *this) {
      p.normalize();
    }
    return *this;
  }

  template <typename ContainerType_>
  template <int idx>
  PointCloud_<ContainerType_>& PointCloud_<ContainerType_>::normalize() {
    for (PointType& p : *this) {
      p.template normalize<idx>();
    }
    return *this;
  }

  template <typename ContainerType_>
  template <TRANSFORM_CLASS transform_class, typename TransformType_>
  PointCloud_<ContainerType_>& PointCloud_<ContainerType_>::transformInPlace(
    const TransformType_& transform_) {
    for (PointType& p : *this) {
      p.template transformInPlace<transform_class, TransformType_>(transform_);
    }
    return *this;
  }

  template <typename ContainerType_>
  template <int idx, TRANSFORM_CLASS transform_class, typename TransformType_>
  PointCloud_<ContainerType_>& PointCloud_<ContainerType_>::transformInPlace(
    const TransformType_& transform_) {
    for (PointType& p : *this) {
      p.template transformInPlace<idx, transform_class, TransformType_>(
        transform_);
    }
    return *this;
  }

  template <typename ContainerType_>
  template <TRANSFORM_CLASS transform_class,
            typename OutputIteratorType_,
            typename TransformType_,
            bool suppress_invalids>
  void PointCloud_<ContainerType_>::transform(
    OutputIteratorType_ dest,
    const TransformType_& transform_) const {
    for (const PointType& src : *this) {
      if (!src.status == Valid) {
        if (!suppress_invalids) {
          *dest++ = src;
        }
        continue;
      }
      PointType dest_p =
        src.template transform<transform_class, TransformType_>(transform_);
      if (dest_p.status == Valid || !suppress_invalids) {
        *dest++ = dest_p;
      }
    }
  }
  
  template <typename ContainerType_>
  template <TRANSFORM_CLASS transform_class, typename TransformType_>
  PointCloud_<ContainerType_> PointCloud_<ContainerType_>::transform(const TransformType_& transform_) const {
    using ThisType=PointCloud_<ContainerType_>;
    using InserterIteratorType = std::back_insert_iterator<ThisType>;
    ThisType returned;
    InserterIteratorType it(returned);
    this->transform<transform_class,InserterIteratorType,TransformType_>(it,transform_);
    return returned;
  }

  template <typename ContainerType_>
  template <int idx,
            TRANSFORM_CLASS transform_class,
            typename OutputIteratorType_,
            typename TransformType_,
            bool suppress_invalids>
  void PointCloud_<ContainerType_>::transform(
    OutputIteratorType_ dest,
    const TransformType_& transform_) const {
    for (const PointType& src : *this) {
      if (!src.status == Valid) {
        if (!suppress_invalids) {
          *dest++ = src;
        }
        continue;
      }
      PointType dest_p =
        src.template transform<idx, transform_class, TransformType_>(
          transform_);
      if (dest_p.status == Valid || !suppress_invalids) {
        *dest++ = dest_p;
      }
    }
  }

  template <typename ContainerType_>
  template <typename OutputIteratorType_>
  void PointCloud_<ContainerType_>::voxelize(OutputIteratorType_ target,
                                             const PlainVectorType& scales) {
    std::vector<VoxelEntry> entries(this->size());
    PlainVectorType inv_scales;
    invertScales(inv_scales, scales);

    // discretize and put in container
    size_t k = 0;
    for (typename ContainerType_::const_iterator it = this->begin();
         it != this->end();
         ++it) {
      if (it->status != Valid) {
        continue;
      }
      entries[k++].set(it, inv_scales);
    }
    if (!k) {
      return;
    }
    entries.resize(k);

    // sort based on the key
    std::sort(entries.begin(), entries.end());

    PointType acc_point = *(entries[0].it);
    int acc_num         = 1;
    for (size_t i = 1; i < k; ++i) {
      const VoxelEntry& entry = entries[i];
      const PointType& point  = *(entry.it);
      // if we pass to the next voxel
      if (entries[i - 1] < entry) {
        // compute the voxelized point
        if (acc_num) {
          acc_point *= (1.f / acc_num);
          acc_point.normalize();
          if (acc_point.status == Valid) {
            *target++ = acc_point;
          }
        }
        // then start accumulating on the new point
        acc_point = point;
        acc_num   = 1;
        continue;
      }
      // otherwise accumulate on the same point
      acc_point += point;
      ++acc_num;
    }
    // add last finalize last point
    if (acc_num) {
      acc_point *= (1.f / acc_num);
      acc_point.normalize();
      if (acc_point.status == Valid) {
        *target++ = acc_point;
      }
    }
  }

  template <typename ContainerType_>
  template <typename OutputIteratorType_>
  void PointCloud_<ContainerType_>::clip(OutputIteratorType_ target,
                                         const PointType& origin,
                                         const PlainVectorType& scales) {
    PlainVectorType plain_origin;
    origin.toPlainVector(plain_origin);
    for (const PointType& p : *this) {
      PlainVectorType plain;
      p.toPlainVector(plain);
      plain -= plain_origin;
      bool keep = true;
      for (int d = 0; d < PointType::Dimensions && keep; ++d) {
        keep = (scales[d] <= 0) || fabs(plain[d]) < scales[d];
      }
      if (keep) {
        *target++ = p;
      }
    }
  }

} // namespace srrg2_core
