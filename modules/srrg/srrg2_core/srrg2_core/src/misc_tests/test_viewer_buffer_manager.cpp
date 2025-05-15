#include "srrg_viewer/viewer_core/buffer_manager.h"
#include "srrg_viewer/viewer_core/packet_deserializer.h"
#include "srrg_viewer/viewer_core/packet_serializer.h"
#include <iostream>

using namespace std;
using namespace srrg2_core;

int main(int argc, char** argv) {
  BufferManager manager;
  manager.param_max_buffer_size.setValue(1024 * 1024);
  manager.param_max_num_buffers.setValue(10);
  manager.init();

  // ia popolate things
  const int num_points = 10;
  Vector3f* points     = new Vector3f[num_points];
  Vector3f* normals    = new Vector3f[num_points];
  for (size_t i = 0; i < num_points; ++i) {
    points[i]  = Vector3f::Ones() * i;
    normals[i] = points[i];
  }

  // ia create some packets
  PacketPayloadPoints p_packet(num_points, points);
  PacketPayloadLines ln_packet(num_points, points, normals);
  PacketInfoEndEpoch end_packet;

  // ia create a packet serializer that will serialize things into its buffer
  PacketSerializer serializer;

  std::cerr << "get a free buffer" << std::endl;
  BufferMemory* buffer_send = manager.getBuffer(); // ia a free buffer
  std::cerr << (size_t) buffer_send << std::endl;
  serializer.setBuffer(buffer_send);
  serializer.putPacket(p_packet);
  serializer.putPacket(ln_packet);
  serializer.putPacket(end_packet);

  // ia ready to be shipped
  manager.releaseBuffer(buffer_send);

  // ia get a ready buffer ready
  BufferMemory* buffer_rec = manager.getBuffer(true); // ia a read buffer
  std::cerr << (size_t) buffer_rec << std::endl;

  PacketDeserializer deserializer;
  deserializer.setBuffer(buffer_rec);

  while (true) {
    PacketBase* p_recv = deserializer.getPacket();
    if (p_recv->type == PACKET_TYPE_END_EPOCH) {
      std::cerr << "end epoch packet. buffer end" << std::endl;
      delete p_recv;
      break;
    }

    switch (p_recv->type) {
      case (PACKET_TYPE_POINTS): {
        std::cerr << "received points" << std::endl;
        break;
      }
      case (PACKET_TYPE_LINES): {
        std::cerr << "received lines" << std::endl;
        break;
      }
      case (PACKET_TYPE_SEGMENTS): {
        std::cerr << "received segments" << std::endl;
        break;
      }
      default: { std::cerr << "received madonne" << std::endl; }
    }

    PacketArray* packet = dynamic_cast<PacketArray*>(p_recv);
    for (size_t i = 0; i < packet->num_points; ++i) {
      std::cerr << "point #" << i << ":\t" << packet->points[i].transpose()
                << std::endl;
    }

    for (size_t i = 0; i < packet->num_normals; ++i) {
      std::cerr << "normals #" << i << ":\t" << packet->normals[i].transpose()
                << std::endl;
    }

    std::cerr << "deleting packet" << std::endl;
    delete p_recv;
  }

  delete[] points;
  delete[] normals;

  return 0;
}
