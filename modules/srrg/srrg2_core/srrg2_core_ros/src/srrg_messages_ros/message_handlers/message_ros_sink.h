#pragma once
#include "srrg_messages/message_handlers/message_sink_base.h"
#include "srrg_property/property_vector.h"
#include <ros/ros.h>

namespace srrg2_core_ros {
  /*! @brief sink that takes a srrg message and publishes it on ros
    you must specify both the topic and the name of the srrg_message
    (ordering matters)
  */
  class MessageROSSink : public srrg2_core::MessageSinkBase {
  public:
    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>,
                 topics,
                 "list of topics that you want to advertise",
                 nullptr);

    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>,
                 types,
                 "list of srrg_msg types (topic[i] should be of type types[i])",
                 nullptr);

  public:
    MessageROSSink();
    virtual ~MessageROSSink();

    /*! does the setup and advertises the topics */
    void open();

    /*! shuts down everything */
    void close();

    /*! given a message it converts it into ROS and sends it over ROS node
      @param[in] msg_ the message to broadcast
      @return true if broadcasting succedeed
    */
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;

    bool cmdOpen(std::string& response);
    bool cmdClose(std::string& response);
  private:
    ros::NodeHandle _nh;
    std::map<std::string, ros::Publisher> _publishers;
  }; // namespace srrg2_core_ros

} // namespace srrg2_core_ros
