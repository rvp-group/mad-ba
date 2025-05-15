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


namespace srrg2_core_ros {

  class BufferPublisher : public srrg2_core::Configurable {
  public:
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::BufferManager>, buffer_manager_ptr, "buffer_manager_ptr", 0, 0);
    PARAM(srrg2_core::PropertyString, message_frame_id, "frame id of the message", "/map", 0);
  public:


    virtual ~BufferPublisher() {
      _sequence_num = 0;
      if (_ros_publisher)
        delete _ros_publisher;
    }

    //! @brief inline set methods
    inline void setRosPublisher(ros::Publisher* pub_) {
      _ros_publisher = pub_;
      if (_sequence_num) _sequence_num = 0;
    }

    inline ros::Publisher* rosPub() {return _ros_publisher;}

    //! @brief if there is a publisher, do the magic
    const bool publishBuffer(srrg2_core::BufferMemory* buffer_);

    inline const size_t numSubscribers() const {
      if (_ros_publisher)
        return _ros_publisher->getNumSubscribers();
      return 0;
    }

  protected:
    //! @brief pointer to the actual ros publisher
    ros::Publisher* _ros_publisher = 0;

    //! @brief message seq number
    size_t _sequence_num=0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  using BufferPublisherPtr = std::shared_ptr<BufferPublisher>;

} //ia end namespace srrg2_core_ros

