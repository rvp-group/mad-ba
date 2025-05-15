#pragma once
#include "packet_base.h"
#include "packet_string.h"

namespace srrg2_core {


  static constexpr uint8_t PACKET_TYPE_CREATE_LIST = 0xB0;
  static constexpr uint8_t PACKET_TYPE_BEGIN_LIST = 0xB1;
  static constexpr uint8_t PACKET_TYPE_END_LIST = 0xB2;
  static constexpr uint8_t PACKET_TYPE_DESTROY_LIST = 0xB3;
  static constexpr uint8_t PACKET_TYPE_CALL_LIST = 0xB4;

  using PacketCreateList  = PacketString_<PACKET_TYPE_CREATE_LIST>;
  using PacketBeginList   = PacketString_<PACKET_TYPE_BEGIN_LIST>;
  using PacketEndList     = PacketCommand_<PACKET_TYPE_END_LIST>;
  using PacketCallList    = PacketString_<PACKET_TYPE_CALL_LIST>;
  using PacketDestroyList = PacketString_<PACKET_TYPE_DESTROY_LIST>;

}
