#include <iostream>

#include "srrg_viewer_ros/viewer_core_ros/buffer_sink_ros.h"
#include <srrg_viewer/viewer_manager_base.h>

#include <srrg2_core_ros/ViewerBufferMessage.h> //ia this is in ros now
#include <ros/ros.h>

#include <signal.h>
#include <thread>

using namespace srrg2_core_ros;
using namespace srrg2_core;

void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    std::cerr << FG_RED("shutting down") << std::endl;
    ros::shutdown();
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "giovanni_huong_node");
  ros::NodeHandle nh;
  ros::Rate ros_rate(1);
  ros::Publisher publisher_marker = nh.advertise<ViewerBufferMessage>("/madonne", 100); //ia publisher name, queue size

  signal(SIGINT, sigIntHandler);



  // ia popolate things
  const int num_points = 100;
  Vector3f* points = new Vector3f[num_points];
  Vector3f* normals = new Vector3f[num_points];
  for (int i = 0; i < num_points; ++i) {
    points[i] = Vector3f::Ones()*i;
    normals[i] = points[i];
  }

  // ia create some packets
  PacketPayloadPoints p_packet(num_points, points);
  PacketPayloadLines ln_packet(num_points, points, normals);
  PacketObjectPyramidWireframe pyr_wf_packet(Vector2f(2.0,1.0f));
  PacketTransformMultMatrix transform_pack(Matrix4f::Random());
  PacketInfoEndEpoch end_packet;

  //ia create a buffer
  BufferMemory* buffer_send = new BufferMemory();
  buffer_send->allocate(BUFFER_SIZE_1MEGABYTE);

  //ia create a packet serializer that will serialize things into its buffer
  PacketSerializer* serializer = new PacketSerializer();
  serializer->setBuffer(buffer_send);
  serializer->putPacket(p_packet);
  serializer->putPacket(ln_packet);
  serializer->putPacket(pyr_wf_packet);
  serializer->putPacket(transform_pack);
  serializer->putPacket(end_packet);

  std::cerr << "buffer info\n"
      << "size     = " << buffer_send->size << std::endl
      << "# packs  = " << buffer_send->num_packets << std::endl
      << "# status = " << buffer_send->status << std::endl;


  ViewerBufferMessage msg;
  msg.header.frame_id = "/muori_ora";
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();

  msg.size = buffer_send->size;
  msg.num_packets = buffer_send->num_packets;
  msg.status = buffer_send->status;
  //ia copy like a onion
  msg.data.resize(buffer_send->size);
  std::memcpy(&msg.data[0], buffer_send->buffer_start, buffer_send->size);

  std::cerr << "message well formed\n";

  while(ros::ok()) {
    while (publisher_marker.getNumSubscribers() < 1) {
      if (!ros::ok()) return 0;
      std::cerr << FG_YELLOW("no subscriber") << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    //ia publish the marker
    publisher_marker.publish(msg);

    ros_rate.sleep(); //ia sleep for publisher
  }


  return 0;
}
