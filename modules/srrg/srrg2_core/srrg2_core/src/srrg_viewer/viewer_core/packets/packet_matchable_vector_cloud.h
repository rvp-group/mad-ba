#pragma once
#include "packet_stl_vector.h"
#include "srrg_matchable/matchable.h"
#include "srrg_matchable/visual_matchable.h"

namespace srrg2_core {
  //! @brief packet that contains a vector of visual matchables
  template <uint8_t PacketType_, typename MatchableType_>
  struct PacketVisualMatchableVector_
    : public PacketVector_<PacketType_, MatchableType_> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    template <typename T_>
    friend class PacketCreator_;
    using BaseClass = PacketVector_<PacketType_, MatchableType_>;

    using MatchableType                 = MatchableType_;
    using EntryTypeVector               = typename BaseClass::EntryTypeVector;
    static constexpr uint8_t PacketType = PacketType_;

    //! @brief default ctor (calls base class)
    PacketVisualMatchableVector_(
      const EntryTypeVector* visual_matchable_vector_ = nullptr) :
      BaseClass(visual_matchable_vector_) {
    }

    char* serialize(char* buffer) const override {
      //      return BaseClass::serialize(buffer);
      buffer = putInBuffer(buffer, this->num_elements);
      for (const auto& m : *(this->data_vector)) {
        buffer = putInBuffer(buffer, m.type());
        buffer = putInBuffer(buffer, m.origin());
        buffer = putInBuffer(buffer, m.rotation());
        buffer = putInBuffer(buffer, m.activation());
        buffer = putInBuffer(buffer, m.color());
        buffer = putInBuffer(buffer, m.support().size());

        if (m.support().size()) {
          std::memcpy((void*) buffer,
                      (const void*) m.support().data(),
                      m.support().size() *
                        sizeof(typename MatchableType::ExtentType));
          buffer +=
            m.support().size() * sizeof(typename MatchableType::ExtentType);
        }
      }

      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      //      return BaseClass::deserialize(buffer);

      BaseClass::clear();
      // ia first get the number of elements
      buffer = getFromBuffer(this->num_elements, buffer);

      // ia allocate and copy
      this->data_vector = new EntryTypeVector();
      this->data_vector->reserve(this->num_elements);

      // ia aux variables
      size_t extent_size         = 0;
      MatchableBase::Type m_type = MatchableBase::Type::Point;

      for (size_t m = 0; m < this->num_elements; ++m) {
        buffer = getFromBuffer(m_type, buffer);
        MatchableType matchable(m_type);

        buffer = getFromBuffer(matchable.origin(), buffer);
        buffer = getFromBuffer(matchable.rotation(), buffer);
        buffer = getFromBuffer(matchable.activation(), buffer);
        buffer = getFromBuffer(matchable.color(), buffer);
        buffer = getFromBuffer(extent_size, buffer);

        if (extent_size) {
          matchable.support().resize(extent_size);
          std::memcpy((void*) matchable.support().data(),
                      (const void*) buffer,
                      extent_size * sizeof(typename MatchableType::ExtentType));
          buffer += extent_size * sizeof(typename MatchableType::ExtentType);
        }

        this->data_vector->emplace_back(matchable);
      }
      return buffer;
    }
  };

  // ----------------------------------------------------------------------- //
  // --------------------------- TYPES DEFS -------------------------------- //
  // ----------------------------------------------------------------------- //
  //! @brief matchables
  static constexpr uint8_t PACKET_TYPE_BASE_MATCHABLE_F_VECTOR   = 0x90;
  static constexpr uint8_t PACKET_TYPE_BASE_MATCHABLE_D_VECTOR   = 0x91;
  static constexpr uint8_t PACKET_TYPE_VISUAL_MATCHABLE_F_VECTOR = 0x92;
  static constexpr uint8_t PACKET_TYPE_VISUAL_MATCHABLE_D_VECTOR = 0x93;

  // ----------------------------------------------------------------------- //
  // ------------------- SPECIFIC PACKET DEFINITIONS ----------------------- //
  // ----------------------------------------------------------------------- //

  //! @brief matchables
  using PacketMatchablefVector =
    PacketVector_<PACKET_TYPE_BASE_MATCHABLE_F_VECTOR, Matchablef>;
  using PacketMatchabledVector =
    PacketVector_<PACKET_TYPE_BASE_MATCHABLE_D_VECTOR, Matchabled>;
  using PacketVisualMatchablefVector =
    PacketVisualMatchableVector_<PACKET_TYPE_VISUAL_MATCHABLE_F_VECTOR,
                                 VisualMatchablef>;
  using PacketVisualMatchabledVector =
    PacketVisualMatchableVector_<PACKET_TYPE_VISUAL_MATCHABLE_D_VECTOR,
                                 VisualMatchabled>;
} // namespace srrg2_core
