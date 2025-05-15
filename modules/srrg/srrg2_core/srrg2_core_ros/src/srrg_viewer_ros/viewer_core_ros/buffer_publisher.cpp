#include "buffer_publisher.h"
#include <srrg2_core_ros/ViewerBufferMessage.h> //ia this is in ros now - my custom message

namespace srrg2_core_ros {
  

  const bool BufferPublisher::publishBuffer(srrg2_core::BufferMemory* buffer_) {
    //ia create a message
    //ia serialize the guy
    //ia publish
    //ia release the buffer
    if (!_ros_publisher)
      throw std::runtime_error("BufferPublisher::publishBuffer| no ros publisher set, exiting");

    if (!param_buffer_manager_ptr.value())
      throw std::runtime_error("BufferPublisher::publishBuffer|no buffer manager set, exiting");

    if (!ros::ok() || _ros_publisher->getNumSubscribers() < 1) {
      std::cerr << "BufferPublisher::publishBuffer|ros not ok" << std::endl;
      param_buffer_manager_ptr->freeBuffer(buffer_);
      return false;
    }

    ViewerBufferMessage msg;
    msg.header.frame_id = param_message_frame_id.value();
    msg.header.seq = _sequence_num++;
    msg.header.stamp = ros::Time::now();

    msg.size = buffer_->size;
    msg.num_packets = buffer_->num_packets;
    msg.status = buffer_->status;
    msg.data.resize(buffer_->size);
    std::memcpy(&msg.data[0], buffer_->buffer_start, buffer_->size);

    _ros_publisher->publish(msg);

    param_buffer_manager_ptr->freeBuffer(buffer_);

    return true;
  }

} /* namespace srrg2_core_ros */
