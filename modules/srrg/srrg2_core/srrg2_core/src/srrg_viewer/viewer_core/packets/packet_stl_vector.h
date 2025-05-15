#pragma once
#include <vector>

#include "packet_base.h"

namespace srrg2_core {
  //! @brief class of packets that contain an std::vector of things
  template <uint8_t PacketType_, typename EntryType_>
  struct PacketVector_ : public PacketBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    template <typename T_>
    friend class PacketCreator_;

    using EntryType = EntryType_;
    using EntryTypeVector =
      std::vector<EntryType, Eigen::aligned_allocator<EntryType>>;
    using PayloadType                   = EntryTypeVector;
    static constexpr uint8_t PacketType = PacketType_;

    size_t num_elements          = 0;
    EntryTypeVector* data_vector = nullptr;

    void clear() {
      if (data_vector) {
        delete data_vector;
      }
      num_elements = 0;
      data_vector  = nullptr;
    }

    void allocate(const size_t& size_) {
      num_elements = size_;
      data_vector  = new EntryTypeVector();
      data_vector->resize(num_elements);
      assert(data_vector->size() == num_elements &&
             "[PacketVector_::allocate]|bad allocate");
    }

    // ia default argument for ctor is required for the factory
    PacketVector_(const EntryTypeVector* vector_ = nullptr) :
      PacketBase(PacketType_) {
      if (vector_) {
        num_elements = vector_->size();
        data_vector  = new EntryTypeVector(*vector_);
        assert(data_vector->size() == vector_->size() &&
               "PacketVector_::PacketVector_|bad construction");
      } else {
        clear();
      }
    }

    ~PacketVector_() {
      if (data_vector) {
        delete data_vector;
      }
      num_elements = 0;
    }

    char* serialize(char* buffer) const override {
      // ia first put the size
      buffer = putInBuffer(buffer, num_elements);
      // ia then put the ciaccia
      std::memcpy((void*) buffer,
                  (const void*) data_vector->data(),
                  num_elements * sizeof(EntryType_));
      buffer += num_elements * sizeof(EntryType_);
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      clear();
      // ia first get the number of elements
      buffer = getFromBuffer(num_elements, buffer);

      // ia allocate and copy
      allocate(num_elements);
      std::memcpy((void*) data_vector->data(),
                  (const void*) buffer,
                  num_elements * sizeof(EntryType_));
      buffer += num_elements * sizeof(EntryType_);
      return buffer;
    }
  };

  //! @brief just in case
  template <uint8_t PacketType_>
  using PacketVector3fVector = PacketVector_<PacketType_, Vector3f>;
  template <uint8_t PacketType_>
  using PacketVector4fVector = PacketVector_<PacketType_, Vector4f>;
} // namespace srrg2_core
