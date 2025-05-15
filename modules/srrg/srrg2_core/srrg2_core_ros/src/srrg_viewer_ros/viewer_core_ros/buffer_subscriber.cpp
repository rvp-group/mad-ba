#include "buffer_subscriber.h"

namespace srrg2_core_ros {

  using namespace srrg2_core;

  void BufferSubscriber::setup(ros::NodeHandle* nh_) {
    if (!nh_) throw std::runtime_error("BufferSubscriber::setup|invalid node handler, exiting");

    _node_handler = nh_;
    _subscriber = nh_->subscribe(param_topic_name.value(), 1, &BufferSubscriber::_bufferMessageCallback, this);
  }

  void BufferSubscriber::_bufferMessageCallback(const ViewerBufferMessageConstPtr& msg_) {
    //std::cerr << "BufferSubscriber::_bufferMessageCallback| received a message" << std::endl;
    //ia each time there is a new message
    //ia   - get a free buffer
    //ia   - deserialize message into the buffer
    //ia   - put into the buffer manager as a ready buffer

    if (!_node_handler)
      throw std::runtime_error("BufferSubscriber::receiveBuffer|no ros node handler set, exiting");

    if (!param_buffer_manager_ptr.value())
      throw std::runtime_error("BufferSubscriber::receiveBuffer|no ros BufferManager set, exiting");

    if (!ros::ok()) {
      std::cerr << "BufferSubscriber::receiveBuffer|ros not ok" << std::endl;
      return;
    }

    //ia let the manager do its magic
    BufferMemory* buffer = param_buffer_manager_ptr->getBuffer();

    //ia check if the buffer manager compliant to the sent buffers
    //ia (maybe this should be done in another moment and this info
    //ia should be published in another topic)
    if (msg_->size != buffer->size) {
      std::cerr << "BufferSubscriber::receiveBuffer|received buffer size   = " << msg_->size << std::endl;
      std::cerr << "BufferSubscriber::receiveBuffer|subscriber buffer size = " << buffer->size << std::endl;
      throw std::runtime_error("BufferSubscriber::receiveBuffer|buffer size mismatch, exiting");
    }

    buffer->num_packets = msg_->num_packets;
    std::memcpy(buffer->data, &msg_->data[0], msg_->size);

    //ia let the manager do its magic again
    param_buffer_manager_ptr->releaseBuffer(buffer);
  }

} //ia end namespace srrg2_core_ros
