#pragma once
#include <algorithm>
#include <array>
#include <bitset>
#include <numeric>

#include <opencv2/opencv.hpp>

namespace srrg2_core {

  //! @brief class that holds a descriptor, templetized on the
  //!        descriptor type. DescriptorType_ could be
  //!        - cv::Mat
  //!        - std::bitset<dim> (from dom)
  //!        - std::array<float, dim> (from dom)
  //!        if you want to add descriptor to someone, just derive from here
  template <typename DescriptorType_>
  class DescriptorOwner_ {
  public:
    //! @brief some usings
    using DescriptorType = DescriptorType_;

    DescriptorOwner_() {
      // ia does nothing
    }

    ~DescriptorOwner_() {
      // ia does nothing
    }

    //! @brief accessors
    inline const DescriptorType& descriptor() const {
      return _descriptor;
    }

    inline DescriptorType& descriptor() {
      return _descriptor;
    }

  protected:
    DescriptorType _descriptor;
  };

  //! @brief standard derivation with binary object as descriptor
  struct DescriptorOwnerMatBinary : public DescriptorOwner_<cv::Mat> {
  public:
    using ThisType           = DescriptorOwnerMatBinary;
    using DescriptorType     = cv::Mat;
    static constexpr int Dim = 1;

    DescriptorOwnerMatBinary() {
      this->_descriptor = 0;
    }

    DescriptorOwnerMatBinary(const ThisType& other_) {
      other_._descriptor.copyTo(this->_descriptor);
    }

    DescriptorOwnerMatBinary(const DescriptorType& desc_) {
      desc_.copyTo(this->_descriptor);
    }

    //! @brief computes distance between two guys [placeholder]
    inline const int distance(ThisType& other_) const {
      return 0;
    }

    //[placeholder]
    //! @brief computes distance between another descriptor of same type
    inline const int distance(const DescriptorType& desc_) const {
      return 0;
    }
  };

  //! @brief bitset derivation
  template <int Dim_>
  class DescriptorOwnerBitset_ : public DescriptorOwner_<std::bitset<Dim_>> {
  public:
    using ThisType           = DescriptorOwnerBitset_<Dim_>;
    using DescriptorType     = std::bitset<Dim_>;
    static constexpr int Dim = Dim_;

    DescriptorOwnerBitset_() {
    }

    DescriptorOwnerBitset_(const ThisType& other_) {
      this->descriptor = other_->_descriptor;
    }

    DescriptorOwnerBitset_(const DescriptorType& desc_) {
      this->descriptor = desc_;
    }

    //! @brief computes distance between two guys [placeholder]
    inline const int distance(const ThisType& other_) const {
      // ia TODO
      return (this->descriptor ^ other_.descriptor).count();
    }

    //[placeholder]
    //! @brief computes distance between another descriptor of same type
    inline const int distance(const DescriptorType& desc_) const {
      return (this->descriptor ^ desc_).count();
    }
  };

  //! @brief specialization on 256bit descriptors
  using DescriptorOwnerBitset256 = DescriptorOwnerBitset_<256>;

  //! @brief floating point derivation
  template <int Dim_>
  class DescriptorOwnerFloat_
    : public DescriptorOwner_<std::array<float, Dim_>> {
  public:
    using ThisType           = DescriptorOwnerFloat_<Dim_>;
    using DescriptorType     = std::array<float, Dim_>;
    static constexpr int Dim = Dim_;

    DescriptorOwnerFloat_() {
    }

    DescriptorOwnerFloat_(const ThisType& other_) {
      this->descriptor = other_->_descriptor;
    }

    DescriptorOwnerFloat_(const DescriptorType& desc_) {
      this->descriptor = desc_;
    }

    //! @brief computes distance between two guys [placeholder]
    inline const int distance(const ThisType& other_) const {
      const DescriptorType& other_desc = other_.descriptor;

      // ds TODO move to reduce in C++17
      // ds TODO optimize for architecture
      DescriptorType difference;
      std::set_difference(this->descriptor.begin(),
                          this->descriptor.end(),
                          other_desc.begin(),
                          other_desc.end(),
                          difference.begin());
      return std::accumulate(
        difference.begin(), difference.end(), 0 /*initial distance*/);
    }

    //[placeholder]
    //! @brief computes distance between another descriptor of same type
    inline const int distance(const DescriptorType& desc_) const {
      // ds TODO move to reduce in C++17
      // ds TODO optimize for architecture
      DescriptorType difference;
      std::set_difference(this->descriptor.begin(),
                          this->descriptor.end(),
                          desc_.begin(),
                          desc_.end(),
                          difference.begin());
      return std::accumulate(
        difference.begin(), difference.end(), 0 /*initial distance*/);
    }
  };

  //! @brief specialization on 32-dim descriptors
  using DescriptorOwnerFloat32 = DescriptorOwnerFloat_<32>;

} // namespace srrg2_core
