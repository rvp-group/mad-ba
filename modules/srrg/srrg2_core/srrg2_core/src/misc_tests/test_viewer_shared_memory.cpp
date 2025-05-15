#include <iostream>
#include <thread>
#include "srrg_viewer/viewer_core/buffer_sink_shared.h"
#include "srrg_viewer/viewer_core/buffer_source_shared.h"
#include "srrg_system_utils/system_utils.h"
#include <signal.h>

#define CONSUMER_TIMESTEP 30
#define PRODUCER_TIMESTEP 3*1000
#define MAX_BUFFER_SIZE 1024*1024*20
#define MAX_BUFFER_NUMBER 2
#define NUM_ROUNDS 4
#define POINTS_BULK_SIZE 200000
#define WOLRD_SIZE 100

using namespace srrg2_core;
using namespace std;

std::atomic<bool> run;
static void sigIntHandler(int __attribute__((unused))) {
  cerr << "Got user interrupt, Terimnating" << endl;
  run=false;
}


void producer(BufferManager* buffer_manager, BufferSinkShared* sink, PacketSerializer* serializer, Vector3f* points, Vector3f* normals) {
  double time = 0.0;
  size_t frame = 0;
  while(run) {
//  for (size_t round = 0; round < NUM_ROUNDS; ++round) {
    SystemUsageCounter::tic();
//    std::this_thread::sleep_for(std::chrono::milliseconds(PRODUCER_TIMESTEP));
//    std::cerr << "[producer] new buffer created" << std::endl;

    //ia create a buffer
    BufferMemory* buffer_send = buffer_manager->getBuffer();
//    std::cerr << "[producer] buffer_send size: " << buffer_send->size << std::endl;

    //ia tell the serializer where it has to put the next packets
    serializer->setBuffer(buffer_send);

    // ia create some packets
    PacketPayloadPoints p_packet_0(POINTS_BULK_SIZE, points);
//    PacketPayloadPoints* p_packet_1(POINTS_BULK_SIZE, points, normals);
    PacketPayloadLines ln_packet_0(POINTS_BULK_SIZE, points);
    PacketPayloadLines ln_packet_1(POINTS_BULK_SIZE, points, normals);
    PacketPayloadSegments s_packet_0(POINTS_BULK_SIZE, points);
    PacketInfoEndEpoch end_packet;

    //ia serialize the packet. the packet changes its ownership,
    //ia so i do not have to take care of it anymore
    serializer->putPacket(p_packet_0);
    serializer->putPacket(ln_packet_0);
    serializer->putPacket(ln_packet_1);
    serializer->putPacket(s_packet_0);
    serializer->putPacket(end_packet);

    //ia send the buffer somewhere. the buffer changes ownership,
    //ia so i do not have to take care of it anymore
//    std::cerr << "[producer] producer sends a fuckin buffer with " << buffer_send->num_packets << " packets" << std::endl;
//    std::cerr << "[producer] written " << buffer_send->data - buffer_send->buffer_start << " bytes " << std::endl;
    sink->putBuffer(buffer_send);
//    std::cerr << "[producer] done" << std::endl;
    ++frame;
    time += SystemUsageCounter::toc();
    if (time > 2.0) {
      std::cerr << "[producer] FPS = " << FG_YELLOW((double)(frame/time)) << std::endl;
      frame = 0;
      time = 0.0;
    }
  }

  run = false;
  std::cerr << "[producer] production stopped" << std::endl;
}

void consumer(BufferSourceShared* source_) {
  run = true;
  PacketDeserializer* deserializer = new PacketDeserializer();
  double time = 0.0;
  size_t frame = 0;
  while (run) {
    SystemUsageCounter::tic();
    std::this_thread::sleep_for(std::chrono::milliseconds(CONSUMER_TIMESTEP));
//    std::cerr << std::endl;
//    std::cerr << "[consumer] waiting for a fuckin buffer" << std::endl;
    BufferMemory* buffer_rec = source_->getBufferTimeout();
    if (!buffer_rec) {
//      std::cerr << "[consumer] no buffer received, connection timeout" << std::endl;
      break;
    }
//    std::cerr << "[consumer] got a fuckin buffer" << std::endl;
//    //stampa le cose
//    std::cerr << "[consumer] buffer_rec size: " << buffer_rec->size << std::endl;
//    std::cerr << "[consumer] buffer_rec num packets: " << buffer_rec->num_packets << std::endl;
//    std::cerr << "[consumer] inspection of the packets" << std::endl;

    deserializer->setBuffer(buffer_rec);
    for (size_t i = 0; i < buffer_rec->num_packets; ++i) {
      PacketBase* packet = deserializer->getPacket();

      switch (packet->type) {
      case (PACKET_TYPE_POINTS):
        {
//          std::cerr << "\t   received points" << std::endl;
          PacketPayloadPoints* points_pack = dynamic_cast<PacketPayloadPoints*>(packet);
          delete points_pack;
          break;
        }
      case (PACKET_TYPE_LINES):
        {
//          std::cerr << "\t   received lines" << std::endl;
          PacketPayloadLines* lines_pack = dynamic_cast<PacketPayloadLines*>(packet);
          delete lines_pack;
          break;
        }
      case (PACKET_TYPE_SEGMENTS):
        {
//          std::cerr << "\t   received segments" << std::endl;
          PacketPayloadSegments* segments_pack = dynamic_cast<PacketPayloadSegments*>(packet);
          delete segments_pack;
          break;
        }
      case (PACKET_TYPE_END_EPOCH):
        {
//          std::cerr << "\t   end epoch" << std::endl;
          PacketInfoEndEpoch* end_pack = dynamic_cast<PacketInfoEndEpoch*>(packet);
          delete end_pack;
          break;
        }
      default:
        {
//          std::cerr << "\t   received madonne" << std::endl;
          delete packet;
        }
      }
    }

    source_->releaseBuffer(buffer_rec); //ia read finished - release the buffer
    ++frame;
    time += SystemUsageCounter::toc();
    if (time > 5.0) {
      std::cerr << "[consumer] FPS = " << FG_GREEN((double)(frame/time)) << std::endl;
      frame = 0;
      time = 0.0;
    }
  }

  std::cerr << "[consumer] stopped" << std::endl;

  delete deserializer;
}

int main (int argc, char** argv) {

  signal(SIGINT, sigIntHandler);

  // ia popolate things
  Vector3f* points = new Vector3f[POINTS_BULK_SIZE];
  Vector3f* normals = new Vector3f[POINTS_BULK_SIZE];
  for (size_t i = 0; i < POINTS_BULK_SIZE; ++i) {
    points[i] = Vector3f::Random()*WOLRD_SIZE;
    normals[i] = points[i];
  }

  std::cerr << "current setup -> \n";
  std::cerr << "POINTS_BULK_SIZE: " << POINTS_BULK_SIZE << "\n";
  std::cerr << "MAX_BUFFER_SIZE: " << MAX_BUFFER_SIZE << "\n";
  std::cerr << "MAX_BUFFER_NUMBER: " << MAX_BUFFER_NUMBER << "\n";
  std::cerr << "PRODUCER TIMESTEP: " << "-" << " ms\n";
  std::cerr << "CONSUMER TIMESTEP: " << CONSUMER_TIMESTEP << " ms\n";

  //ia create a packet serializer that will serialize things into its buffer
  PacketSerializer* serializer = new PacketSerializer();

  //ia create and initialize a buffer manager with a certain amount of buffers;
  //ia this must be set both in the sink and in the source (they share this object)
  BufferManagerPtr buffer_manager(new BufferManager());
  buffer_manager->param_max_num_buffers.setValue(MAX_BUFFER_NUMBER);
  buffer_manager->param_max_buffer_size.setValue(MAX_BUFFER_SIZE);
  buffer_manager->init();

  //ia should go in another executable 
  //ia create a sink where I can put the buffer and send it to the moon
  BufferSinkSharedPtr sink(new BufferSinkShared());
  BufferSourceSharedPtr source(new BufferSourceShared());
  sink->param_source_ptr.setValue(source);
  sink->param_manager_ptr.setValue(buffer_manager);
  source->param_manager_ptr.setValue(buffer_manager);

  //ia producer cycle
  run = true;
  //ia start the consumer thread
  std::thread producer_t(producer, buffer_manager.get(), sink.get(), serializer, points, normals);

  //ia start the consumer in the main
  consumer(source.get());

  producer_t.join();


  delete serializer;

  delete [] normals;
  delete [] points;
  return 0;
}
