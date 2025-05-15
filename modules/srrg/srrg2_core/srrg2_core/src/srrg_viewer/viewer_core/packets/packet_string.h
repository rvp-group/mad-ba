#pragma once
#include "packet_base.h"

namespace srrg2_core {
  //! @breif maxt text is 255 characters
  static constexpr uint8_t TEXT_MAX_LENGTH = 0xFF;

  //! @brief packet that contains text
  struct PacketString : PacketBase {
    char data[TEXT_MAX_LENGTH];
    PacketString(const uint8_t type_, const std::string& field_ = "") :
      PacketBase(type_) {
      assert(field_.length() <= TEXT_MAX_LENGTH &&
             "PacketString::PacketString|text too long");
      std::strcpy(data, field_.c_str());
    }

    virtual ~PacketString() {
    }

    char* serialize(char* buffer) const override {
      //      buffer=putInBuffer(buffer, info);
      std::memcpy(
        (void*) buffer, (const void*) data, TEXT_MAX_LENGTH * sizeof(char));
      buffer += TEXT_MAX_LENGTH * sizeof(char);
      return buffer;
    }

    const char* deserialize(const char* buffer) override {
      //      buffer = getFromBuffer(info, buffer);
      std::memcpy(data, (const void*) buffer, TEXT_MAX_LENGTH * sizeof(char));
      buffer += TEXT_MAX_LENGTH * sizeof(char);
      return buffer;
    }
  };

  template <uint8_t PacketType_>
  struct PacketString_ : PacketString {
    static constexpr uint8_t PacketType = PacketType_;
    PacketString_(const std::string& field_ = "") :
      PacketString(PacketType_, field_) {
    }
  };

  static constexpr uint8_t PACKET_TYPE_TEXT = 0xA0;

  //@brief data -> string(text)
  using PacketObjectText  = PacketString_<PACKET_TYPE_TEXT>;


} // namespace srrg2_core
