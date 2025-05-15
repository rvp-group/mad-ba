#include "srrg_viewer/viewer_core/packet_factory.h"
#include <iostream>

using namespace srrg2_core;
using namespace std;

int main(int argc, char** argv) {
  PacketFactory factory;

  PacketInfoEndEpoch* p0 =
    dynamic_cast<PacketInfoEndEpoch*>(factory.createPacket(PACKET_TYPE_END_EPOCH));
  if (!p0) {
    std::cerr << std::printf("factor error, failed to create packet [%02X]", PACKET_TYPE_END_EPOCH);
    throw std::runtime_error("exit");
  }
  delete p0;

  PacketPointIntensityDescriptor2fVectorCloud* p1 =
    dynamic_cast<PacketPointIntensityDescriptor2fVectorCloud*>(
      factory.createPacket(PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_2F_VECTOR));
  if (!p1) {
    std::cerr << std::printf("factor error, failed to create packet [%02X]",
                             PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_2F_VECTOR);
    throw std::runtime_error("exit");
  }
  delete p1;

  PacketVisualMatchabledVector* p2 = dynamic_cast<PacketVisualMatchabledVector*>(
    factory.createPacket(PACKET_TYPE_VISUAL_MATCHABLE_D_VECTOR));
  if (!p2) {
    std::cerr << std::printf("factor error, failed to create packet [%02X]",
                             PACKET_TYPE_VISUAL_MATCHABLE_D_VECTOR);
    throw std::runtime_error("exit");
  }
  delete p2;

  std::printf("created packet [%02X]\n", PACKET_TYPE_END_EPOCH);
  std::printf("created packet [%02X]\n", PACKET_TYPE_POINT_INTENSITY_DESCRIPTOR_2F_VECTOR);
  std::printf("created packet [%02X]\n", PACKET_TYPE_VISUAL_MATCHABLE_D_VECTOR);

  return 0;
}
