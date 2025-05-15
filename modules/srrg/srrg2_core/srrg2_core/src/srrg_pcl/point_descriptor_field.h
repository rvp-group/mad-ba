#pragma once
#include <array>
#include <bitset>
#include <numeric>
#include <opencv2/core.hpp>

#include "point_default_field.h"
#include "srrg_system_utils/stream_helpers.h"

namespace srrg2_core {

  //! traits that define the available operations on a descriptor field
  template <typename PointFieldType_>
  struct PointDescriptorFieldTraits_ : PointDefaultFieldTraits_<PointFieldType_> {
    using PointFieldType = PointFieldType_;

    //! dimension of the descriptor field
    //! the number of bits for a binary descriptor (stored as vector of bits)
    //! the number of floats for a floating-point descriptor (stored as a vector
    //! of floats)
    static constexpr int Dim = PointFieldType::Dim;

    //! no operations supported
  };

  //! deprecated
  struct PointDescriptorFieldCvMatBinary {
    static constexpr int Dim = 1; // ds INVALID for cv::Mat
    using ValueType          = cv::Mat;
    using TraitsType         = PointDescriptorFieldTraits_<ValueType>;
    ValueType value;

    //! similarity distance function for binary descriptors
    const int distance(const PointDescriptorFieldCvMatBinary& other_) const;
  };

  //! deprecated
  struct PointDescriptorFieldCvMatFloat {
    static constexpr int Dim = 1; // ds INVALID for cv::Mat
    using ValueType          = cv::Mat;
    using TraitsType         = PointDescriptorFieldTraits_<ValueType>;
    ValueType value;

    //! similarity distance function for floating point descriptors
    const int distance(const PointDescriptorFieldCvMatFloat& other_) const;
  };

  //! 32 byte binary descriptor (i.e. dimension is defined as number of bits)
  struct PointDescriptorFieldBit256 {
    static constexpr int Dim = 256;
    using ValueType          = std::bitset<256>;
    using TraitsType         = PointDescriptorFieldTraits_<ValueType>;
    ValueType value;

    //! similarity distance function for binary descriptors
    const int distance(const PointDescriptorFieldBit256& other_) const;

    //! conversion from another descriptor field type
    void from(const PointDescriptorFieldCvMatBinary& other_);

    //! set binary descriptor data from raw cv::Mat descriptor
    void from(const cv::Mat& other_value_);
  };

  //! 32 (x4 byte on most archs) floating point descriptor
  struct PointDescriptorFieldFloat32 {
    static constexpr int Dim = 32;
    using ValueType          = std::array<float, 32>;
    using TraitsType         = PointDescriptorFieldTraits_<ValueType>;
    ValueType value;

    //! similarity distance function for floating point descriptors
    const int distance(const PointDescriptorFieldFloat32& other_) const;
  };

} // namespace srrg2_core
