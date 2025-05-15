#include <iostream>

#include "srrg_viewer_ros/viewer_core_ros/buffer_sink_ros.h"
#include <srrg_viewer/viewer_manager_base.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg2_core_ros/ViewerBufferMessage.h> //ia this is in ros now
#include <ros/ros.h>

#include <signal.h>
#include <thread>

using namespace srrg2_core_ros;
using namespace srrg2_core;

#define COMMON_BUFFER_SIZE 1024*1024
#define POINTS_BULK_SIZE 100
#define PRODUCER_TIMESTEP 30
#define TOPIC_NAME "/buffer_messages"


void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    std::cerr << FG_RED("shutting down") << std::endl;
    ros::shutdown();
  }
}


void producer(BufferManagerPtr buffer_manager, BufferSinkRos* sink, PacketSerializer* serializer);


int main(int argc, char **argv) {
  std::cerr << "[main] test buffer sink ros parameters\n"
            << "\t buffer size    = " << FG_CYAN(COMMON_BUFFER_SIZE) << std::endl
            << "\t topic name     = " << FG_CYAN(TOPIC_NAME) << std::endl
            << "\t producer sleep = " << FG_CYAN(PRODUCER_TIMESTEP) << " ms" << std::endl;

  signal(SIGINT, sigIntHandler);

  ros::init(argc, argv, "test_buffer_sink_node");
  ros::NodeHandle nh;
  ros::Rate ros_rate(1);

  PacketSerializer* serializer = new PacketSerializer();

  BufferManagerPtr manager = BufferManagerPtr(new BufferManager);
  manager->param_max_buffer_size.setValue(COMMON_BUFFER_SIZE);
  manager->param_max_num_buffers.setValue(10);
  manager->init();

  ros::Publisher rp = nh.advertise<ViewerBufferMessage>("/buffer_messages", 100);
  BufferPublisherPtr publisher = BufferPublisherPtr(new BufferPublisher);
  publisher->setRosPublisher(&rp);
  publisher->param_buffer_manager_ptr.setValue(manager);

  BufferSinkRos* sink = new BufferSinkRos();
  sink->param_manager_ptr.setValue(manager);
  sink->param_publisher_ptr.setValue(publisher);

  std::thread producer_t(producer, manager, sink, serializer);

  producer_t.join();

  return 0;
}


void producer(BufferManagerPtr buffer_manager, BufferSinkRos* sink, PacketSerializer* serializer) {
  // ia popolate things
  Vector3f* points = new Vector3f[POINTS_BULK_SIZE];
  Vector3f* normals = new Vector3f[POINTS_BULK_SIZE];
  for (int i = 0; i < POINTS_BULK_SIZE; ++i) {
    points[i] = Vector3f::Ones()*i;
    normals[i] = points[i];
  }

  double time = 0.0;
  size_t frame = 0;
  while(ros::ok()) {
    //  for (size_t round = 0; round < NUM_ROUNDS; ++round) {
    SystemUsageCounter::tic();
    std::this_thread::sleep_for(std::chrono::milliseconds(PRODUCER_TIMESTEP));

    //ia create a buffer
    BufferMemory* buffer_send = buffer_manager->getBuffer();

    //ia tell the serializer where it has to put the next packets
    serializer->setBuffer(buffer_send);

    // ia create some packets
    PacketPayloadPoints p_packet_0(POINTS_BULK_SIZE, points);
    //    PacketPayloadPoints* p_packet_1 = new PacketPayloadPoints(POINTS_BULK_SIZE, points, normals);
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

    std::cerr << "[producer] buffer info\n"
        << "\tsize     = " << buffer_send->size << std::endl
        << "\t# packs  = " << buffer_send->num_packets << std::endl
        << "\t# status = " << buffer_send->status << std::endl;
    sink->putBuffer(buffer_send);


    ++frame;
    time += SystemUsageCounter::toc();
    if (time > 2.0) {
      std::cerr << "[producer] FPS = " << FG_YELLOW((double)(frame/time)) << std::endl;
      frame = 0;
      time = 0.0;
    }
  }

  std::cerr << "[producer] production stopped" << std::endl;

  delete[] points;
  delete[] normals;
}

