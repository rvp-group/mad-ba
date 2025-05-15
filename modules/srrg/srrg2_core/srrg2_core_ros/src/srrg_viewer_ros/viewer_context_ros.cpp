#include "viewer_context_ros.h"

namespace srrg2_core_ros {

  ViewerContextRos::ViewerContextRos(const std::string& context_name_,
                                     srrg2_core::ViewerManagerBase* manager_,
                                     const size_t& num_buffers_,
                                     const size_t& buffer_size_,
                                     ros::NodeHandle* node_) :
                                         ViewerContextBase(context_name_,
                                                           manager_,
                                                           num_buffers_,
                                                           buffer_size_)
  {
    if (!node_)
      throw std::runtime_error("ViewerContextRos::ViewerContextRos|invalid ros node");
    //ia create shared memory substratum
    _type = ViewerContextType::ROS;
    _source.reset(new BufferSourceRos());
    _sink.reset(new BufferSinkRos());

    //ia context is created but not binded
    _status = ViewerContextStatus::Inactive;

    //ia ros things
    _num_subscribers = 0;
    _node = node_;
  }


  ViewerContextRos::~ViewerContextRos() {
    _status = ViewerContextStatus::Invalid;
  }


  void ViewerContextRos::setup() {
    if (_status != ViewerContextStatus::Inactive)
      throw std::runtime_error("ViewerContextRos::setup|current context is already in use or not well formed, exit");
    if (_type != ViewerContextType::ROS)
      throw std::runtime_error("ViewerContextRos::setup|current context is not ROS, exit");

    _canvas->param_buffer_sink_ptr.setValue(_sink);
    _canvas->param_buffer_manager_ptr.setValue(_buffer_manager);

    _status = ViewerContextStatus::Ready;
  }


  void ViewerContextRos::_setupPublisher(const std::string& topic_name_, const std::string& frame_id_name_) {
    BufferSinkRosPtr ros_sink = std::dynamic_pointer_cast<BufferSinkRos>(_sink);
    if (!ros_sink)
      throw std::runtime_error("ViewerContextRos::_setupPublisher|invalid sink");

    ros::Publisher* rp = new ros::Publisher(_node->advertise<ViewerBufferMessage>(topic_name_, 1));
    _publisher.reset(new BufferPublisher);
    _publisher->setRosPublisher(rp);
    _publisher->param_buffer_manager_ptr.setValue(_buffer_manager);
    _publisher->param_message_frame_id.setValue(frame_id_name_);

    ros_sink->param_manager_ptr.setValue(_buffer_manager);
    ros_sink->param_publisher_ptr.setValue(_publisher);
  }

  void ViewerContextRos::_setupSubscriber(const std::string& topic_name_) {
    BufferSourceRosPtr ros_source = std::dynamic_pointer_cast<BufferSourceRos>(_source);
    if (!ros_source)
      throw std::runtime_error("ViewerContextRos::_setupSubscriber|invalid source");

    _subscriber.reset(new BufferSubscriber);
    _subscriber->param_buffer_manager_ptr.setValue(_buffer_manager);
    _subscriber->param_topic_name.setValue(topic_name_);
    _subscriber->setup(_node);

    ros_source->param_manager_ptr.setValue(_buffer_manager);
    ros_source->param_subscriber_ptr.setValue(_subscriber);


  }
} //ia end namespace srrg2_core_ros
