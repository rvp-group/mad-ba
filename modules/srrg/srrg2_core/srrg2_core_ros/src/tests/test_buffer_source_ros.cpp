#include <iostream>
#include <thread>

#include <signal.h>

#include <srrg_viewer/viewer_manager_base.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg_viewer_ros/viewer_core_ros/buffer_source_ros.h"

#define COMMON_BUFFER_SIZE 1024*1024*10
#define CONSUMER_TIMESTEP 30
#define TOPIC_NAME "/buffer_topic"

using namespace srrg2_core_ros;
using namespace srrg2_core;

void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    std::cerr << FG_RED("shutting down") << std::endl;
    ros::shutdown();
  }
}

void consumer(BufferSourceRos* source_);

int main(int argc, char **argv) {
  std::cerr << "[main] test buffer source ros parameters\n"
            << "\t buffer size    = " << FG_CYAN(COMMON_BUFFER_SIZE) << std::endl
            << "\t topic name     = " << FG_CYAN(TOPIC_NAME) << std::endl
            << "\t producer sleep = " << FG_CYAN(CONSUMER_TIMESTEP) << " ms" << std::endl;

  ros::init(argc, argv, "test_buffer_source_node");
  ros::NodeHandle nh;

  signal(SIGINT, sigIntHandler);

  //ia setup
  BufferManagerPtr manager = BufferManagerPtr(new BufferManager);
  manager->param_max_buffer_size.setValue(COMMON_BUFFER_SIZE);
  manager->param_max_num_buffers.setValue(10);
  manager->init();

  BufferSubscriberPtr listner = BufferSubscriberPtr(new BufferSubscriber);
  listner->param_topic_name.setValue(TOPIC_NAME);
  listner->param_buffer_manager_ptr.setValue(manager);
  listner->setup(&nh);

  BufferSourceRos* source = new BufferSourceRos();
  source->param_manager_ptr.setValue(manager);
  source->param_subscriber_ptr.setValue(listner);

  //ia this is the right order to make things work:
  //ia 1. start the consumer thread
  //ia 2. make ros spin forever
  //ia 3. join the thread and shutdown things
  std::thread consumer_t(consumer, source);

  ros::spin();
  consumer_t.join();

  delete source;
  return 0;
}

void consumer(BufferSourceRos* source_) {
  PacketDeserializer* deserializer = new PacketDeserializer();
  double time = 0.0;
  size_t frame = 0;
  while (ros::ok()) {
    SystemUsageCounter::tic();
    std::this_thread::sleep_for(std::chrono::milliseconds(CONSUMER_TIMESTEP));
    //    std::cerr << std::endl;
    //    std::cerr << "[consumer] waiting for a fuckin buffer" << std::endl;
    BufferMemory* buffer_rec = source_->getBuffer();
    if (!buffer_rec) {
      std::cerr << "[consumer] no buffer received" << std::endl;
      break;
    }
    std::cerr << "[consumer] received buffer has " << buffer_rec->num_packets << " packets" << std::endl;


    deserializer->setBuffer(buffer_rec);
    for (size_t i = 0; i < buffer_rec->num_packets; ++i) {
      PacketBase* packet = deserializer->getPacket();

      switch (packet->type) {
      case (PACKET_TYPE_POINTS):
      {
        std::cerr << "\t   received points" << std::endl;
        PacketPayloadPoints* points_pack = dynamic_cast<PacketPayloadPoints*>(packet);
        delete points_pack;
        break;
      }
      case (PACKET_TYPE_LINES):
      {
        std::cerr << "\t   received lines" << std::endl;
        PacketPayloadLines* lines_pack = dynamic_cast<PacketPayloadLines*>(packet);
        delete lines_pack;
        break;
      }
      case (PACKET_TYPE_SEGMENTS):
      {
        std::cerr << "\t   received segments" << std::endl;
        PacketPayloadSegments* segments_pack = dynamic_cast<PacketPayloadSegments*>(packet);
        delete segments_pack;
        break;
      }
      case (PACKET_TYPE_END_EPOCH):
      {
        std::cerr << "\t   end epoch" << std::endl;
        PacketInfoEndEpoch* end_pack = dynamic_cast<PacketInfoEndEpoch*>(packet);
        delete end_pack;
        break;
      }
      default:
      {
        std::cerr << "\t   received something else" << std::endl;
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
