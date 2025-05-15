#pragma once
//ia properties stuff
#include <srrg_property/property_container.h>
#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
//ia buffer stuff
#include <srrg_viewer/viewer_core/buffer_memory.h>
#include <srrg_viewer/viewer_core/buffer_manager.h>
//ia ros
#include <ros/ros.h>
#include <srrg2_core_ros/ViewerBufferMessage.h> //ia this is in ros now

namespace srrg2_core_ros {

  class BufferSubscriber : public srrg2_core::Configurable {
  public:
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::BufferManager>, buffer_manager_ptr, "buffer_manager_ptr", 0,0);
    PARAM(srrg2_core::PropertyString, topic_name, "topic name where buffer are published", "/buffer_messages",0);

  public:

    virtual ~BufferSubscriber() {};

    //! @brief setup the listner and subscribe to
    //!        the topic specified in the config
    void setup(ros::NodeHandle* nh_ = 0);

  protected:
    //! @brief callback to handle messages
    void _bufferMessageCallback(const ViewerBufferMessageConstPtr& msg_);

    //! @brief just in case, not useful here
    ros::NodeHandle* _node_handler = 0;
    ros::Subscriber _subscriber;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  using BufferSubscriberPtr = std::shared_ptr<BufferSubscriber>;
  using BufferSubscriberPtrVector = std::vector<BufferSubscriberPtr>;

} //ia end namespace srrg2_core_ros

