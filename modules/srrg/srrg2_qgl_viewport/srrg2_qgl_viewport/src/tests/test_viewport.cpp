#include <chrono>
#include <iostream>
#include <thread>

#include <srrg_viewer/viewer_core/buffer_sink_shared.h>
#include <srrg_viewer/viewer_core/buffer_source_shared.h>

#include "srrg_qgl_viewport/qgl_viewport.h"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace std;

#define SLEEP_MS_PRODUCER 30
#define MAX_BUFFER_SIZE 1024 * 1024 * 256
#define MAX_BUFFER_NUMBER 10

void producer(BufferSinkSharedPtr sink_);

int main(int argc, char** argv) {
  QApplication qapp(argc, argv);

  //! setup things
  BufferManagerPtr manager(new BufferManager());
  manager->param_max_buffer_size.setValue(MAX_BUFFER_SIZE);
  manager->param_max_num_buffers.setValue(MAX_BUFFER_NUMBER);
  manager->init();

  BufferSourceSharedPtr source(new BufferSourceShared());
  BufferSinkSharedPtr sink(new BufferSinkShared());
  sink->param_source_ptr.setValue(source);
  sink->param_manager_ptr.setValue(manager);
  source->param_manager_ptr.setValue(manager);

  // ia secondary thread produces data since the damn qgl must go in
  // ia the main one otherwise it will complain a lot
  std::thread producer_t(producer, sink);

  // ia damn qgl must go in the main thread
  // ia setup things
  QGLViewport viewport;
  viewport.param_source.setValue(source);
  viewport.setQGLServer(&qapp);

  while (viewport.isActive()) {
    viewport.update();
  }

  // ia when I close the viewport I disconnect the sink and the source
  sink->param_source_ptr.setValue(0);

  // ia cleanup
  producer_t.join();

  return 0;
}

void producer(BufferSinkSharedPtr sink_) {
  bool run                     = true;
  BufferManagerPtr manager_ptr = sink_->param_manager_ptr.value();

  // ia popolate things
  const int num_points = 1000;
  Vector3f* points     = new Vector3f[num_points];
  Vector3f* normals    = new Vector3f[num_points];
  for (int i = 0; i < num_points; ++i) {
    points[i]  = Vector3f::Ones() * i;
    normals[i] = Vector3f::UnitZ();
  }

  // ia now put things in the buffer
  PacketSerializer* serializer = new PacketSerializer();

  while (run) {
    // ia set a buffer where the packets will go
    BufferMemory* buffer = manager_ptr->getBuffer();
    serializer->setBuffer(buffer);

    // ia create some packets and serialize the packet. it changes its ownership,
    // ia so i do not have to take care of it anymore
    serializer->putPacket(PacketCommandPushColor());

    serializer->putPacket(PacketAttributeColorRGB(Vector3f(0.7, 0.3, 0.1)));

    serializer->putPacket(PacketPayloadPoints(num_points, points));

    serializer->putPacket(PacketCommandPopAttribute());

    serializer->putPacket(PacketCommandPushColor());

    serializer->putPacket(PacketAttributeColorRGB(Vector3f(0.1, 0.7, 0.1)));

    serializer->putPacket(PacketPayloadLines(num_points, points, normals));

    serializer->putPacket(PacketCommandPopAttribute());

    serializer->putPacket(PacketInfoEndEpoch());

    std::cerr << "[producer] new buffer ready to be sent" << std::endl;

    // ia send the buffer somewhere. the buffer changes ownership,
    // ia so i do not have to take care of it anymore
    std::cerr << "[producer] producer sends a fuckin buffer with " << buffer->num_packets
              << " packets" << std::endl;
    sink_->putBuffer(buffer);
    std::cerr << "[producer] done, sleeping a bit" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS_PRODUCER));
  }

  // ia cleanup;
  delete serializer;
  delete[] points;
  delete[] normals;
}
