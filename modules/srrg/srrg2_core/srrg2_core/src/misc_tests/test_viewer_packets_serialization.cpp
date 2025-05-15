#include <iostream>

#include "srrg_viewer/viewer_core/packets.h"

#include "srrg_viewer/viewer_core/packet_deserializer.h"
#include "srrg_viewer/viewer_core/packet_serializer.h"

using namespace srrg2_core;
using namespace std;

int main(int argc, char** argv) {
  static constexpr uint64_t maximum_buffer_size = 1024 * 1024 * 256;

  // ia popolate things
  const int num_points = 100;
  Vector3f* points     = new Vector3f[num_points];
  Vector3f* normals    = new Vector3f[num_points];
  for (int i = 0; i < num_points; ++i) {
    points[i]  = Vector3f::Ones() * i;
    normals[i] = points[i];
  }

  // ia matchables
  const size_t num_matchables = 20;
  MatchablefVector matchable_vector;
  matchable_vector.reserve(num_matchables);
  VisualMatchablefVector visual_matchable_vector;
  visual_matchable_vector.reserve(num_matchables);
  for (size_t m = 0; m < num_matchables; ++m) {
    Vector3f origin = Vector3f::Zero() + Vector3f::Ones() * m;
    std::cerr << "origin = " << origin.transpose() << std::endl;

    Matchablef matchable(MatchableBase::Type::Plane, origin);
    matchable_vector.emplace_back(matchable);

    VisualMatchablef v_matchable(MatchableBase::Type::Plane, origin);
    v_matchable.color() = ColorPalette::color3fBlue();
    v_matchable.support().reserve(1000);
    for (size_t i = 0; i < 1000; ++i) {
      PointNormalCurvatureColor3f extent_point;
      extent_point.coordinates() = Vector3f::Random();
      extent_point.normal()      = Vector3f::UnitX();
      extent_point.color()       = v_matchable.color();
      extent_point.curvature()   = 0.1f;
      v_matchable.support().emplace_back(extent_point);
    }

    visual_matchable_vector.emplace_back(v_matchable);
  }

  // ia create some packets
  PacketPayloadPoints p_packet(num_points, points);
  PacketPayloadLines ln_packet(num_points, points, normals);
  PacketObjectPyramidWireframe pyr_wf_packet(Vector2f(2.0, 1.0f));
  PacketTransformMultMatrix transform_pack(Matrix4f::Random());
  std::cerr << "input\n" << transform_pack.data << std::endl;
  PacketMatchablefVector matchable_pack(&matchable_vector);
  PacketVisualMatchablefVector visual_matchable_pack(&visual_matchable_vector);
  PacketInfoEndEpoch end_packet;

  // ia create a buffer
  BufferMemory* buffer_send = new BufferMemory();
  buffer_send->allocate(maximum_buffer_size);

  // ia create a packet serializer that will serialize things into its buffer
  PacketSerializer* serializer = new PacketSerializer();
  serializer->setBuffer(buffer_send);
  serializer->putPacket(p_packet);
  serializer->putPacket(ln_packet);
  serializer->putPacket(pyr_wf_packet);
  serializer->putPacket(transform_pack);
  serializer->putPacket(matchable_pack);
  serializer->putPacket(visual_matchable_pack);
  serializer->putPacket(end_packet);

  // ia clone the buffer to simulate a socket
  BufferMemory* buffer_rec         = buffer_send->clone();
  PacketDeserializer* deserializer = new PacketDeserializer();
  deserializer->setBuffer(buffer_rec);

  while (true) {
    PacketBase* p_recv = deserializer->getPacket();
    if (p_recv->type == PACKET_TYPE_END_EPOCH) {
      std::cerr << "received PACKET_TYPE_END_EPOCH -> buffer end" << std::endl;
      delete p_recv;
      break;
    }

    switch (p_recv->type) {
      case (PACKET_TYPE_POINTS): {
        std::cerr << "received PACKET_TYPE_POINTS" << std::endl;
        PacketArray* packet = dynamic_cast<PacketArray*>(p_recv);
        for (size_t i = 0; i < packet->num_points; ++i) {
          std::cerr << "point #" << i << ":\t" << packet->points[i].transpose() << std::endl;
        }

        for (size_t i = 0; i < packet->num_normals; ++i) {
          std::cerr << "normals #" << i << ":\t" << packet->normals[i].transpose() << std::endl;
        }
        break;
      }
      case (PACKET_TYPE_LINES): {
        std::cerr << "received PACKET_TYPE_LINES" << std::endl;
        PacketArray* packet = dynamic_cast<PacketArray*>(p_recv);
        for (size_t i = 0; i < packet->num_points; ++i) {
          std::cerr << "point #" << i << ":\t" << packet->points[i].transpose() << std::endl;
        }

        for (size_t i = 0; i < packet->num_normals; ++i) {
          std::cerr << "normals #" << i << ":\t" << packet->normals[i].transpose() << std::endl;
        }
        break;
      }
      case (PACKET_TYPE_SEGMENTS): {
        std::cerr << "received PACKET_TYPE_SEGMENTS" << std::endl;
        PacketArray* packet = dynamic_cast<PacketArray*>(p_recv);
        for (size_t i = 0; i < packet->num_points; ++i) {
          std::cerr << "point #" << i << ":\t" << packet->points[i].transpose() << std::endl;
        }

        for (size_t i = 0; i < packet->num_normals; ++i) {
          std::cerr << "normals #" << i << ":\t" << packet->normals[i].transpose() << std::endl;
        }
        break;
      }
      case (PACKET_TYPE_PYRAMID_WIREFRAME): {
        std::cerr << "received PACKET_TYPE_PYRAMID_WIREFRAME" << std::endl;
        std::cerr << "object info: "
                  << dynamic_cast<PacketObjectPyramidWireframe*>(p_recv)->data.transpose()
                  << std::endl;
        break;
      }
      case (PACKET_TYPE_MULT_MATRIX4F): {
        std::cerr << "received PACKET_TYPE_MULT_MATRIX4F" << std::endl;
        std::cerr << "transform data:\n"
                  << dynamic_cast<PacketTransformMultMatrix*>(p_recv)->data << std::endl;
        break;
      }
      case (PACKET_TYPE_BASE_MATCHABLE_F_VECTOR): {
        std::cerr << "received PACKET_TYPE_BASE_MATCHABLE_F_VECTOR" << std::endl;
        PacketMatchablefVector* packet = dynamic_cast<PacketMatchablefVector*>(p_recv);
        for (size_t i = 0; i < packet->num_elements; ++i) {
          std::cerr << "origin #" << i << ":\t" << packet->data_vector->at(i).origin().transpose()
                    << std::endl;
        }
        break;
      }
      case (PACKET_TYPE_VISUAL_MATCHABLE_F_VECTOR): {
        std::cerr << "received PACKET_TYPE_VISUAL_MATCHABLE_F_VECTOR " << std::endl;
        PacketVisualMatchablefVector* packet = dynamic_cast<PacketVisualMatchablefVector*>(p_recv);
        for (size_t i = 0; i < packet->num_elements; ++i) {
          std::cerr << "origin #" << i << ":\t" << packet->data_vector->at(i).origin().transpose()
                    << std::endl;
        }
        break;
      } /***/
      default: { std::cerr << "received madonne" << std::endl; }
    }

    std::cerr << "deleting packet" << std::endl;
    delete p_recv;
  }

  delete deserializer;
  delete serializer;
  delete buffer_rec;
  delete buffer_send;
  delete[] points;
  delete[] normals;
  return 0;
}
