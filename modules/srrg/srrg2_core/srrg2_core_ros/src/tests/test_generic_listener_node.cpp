#include <ros/ros.h>

#include "srrg_data_structures/platform.h"
#include "srrg_converters/converter.h"
#include "srrg_property/property_container.h"
#include "srrg_property/property_serializable.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

//ds register requested SRRG message classes
//BOSS_REGISTER_CLASS(PropertyTransformEventContainer)
BOSS_REGISTER_CLASS(IMUMessage)
BOSS_REGISTER_CLASS(RangeMessage)
BOSS_REGISTER_CLASS(LaserMessage)
BOSS_REGISTER_CLASS(ImageMessage)



//ds stack of global, generic messages in SRRG format, received from ROS
std::list<PropertyContainerSerializablePtr> srrg_messages_to_process;

//ds ROS topic callbacks with fully automatic conversion to generic SRRG message
void callbackTFMessage(const tf2_msgs::TFMessageConstPtr& message_) {
  srrg_messages_to_process.push_back(Converter::convert(message_));
}
void callbackIMU(const sensor_msgs::ImuConstPtr& message_) {
  srrg_messages_to_process.push_back(Converter::convert(message_));
}
void callbackRange(const sensor_msgs::RangeConstPtr& message_) {
  srrg_messages_to_process.push_back(Converter::convert(message_));
}
void callbackLaserScan(const sensor_msgs::LaserScanConstPtr& message_) {
  srrg_messages_to_process.push_back(Converter::convert(message_));
}
void callbackImage(const sensor_msgs::ImageConstPtr& message_) {
  srrg_messages_to_process.push_back(Converter::convert(message_));
}

int main(int argc, char** argv) {

  //ds initialize roscpp
  ros::init(argc, argv, "test_tf_listener");

  //ds start node
  ros::NodeHandle node;
  srrg_messages_to_process.clear();

  //ds subscribe to topics
  ros::Subscriber subscriber_tf           = node.subscribe("/tf", 1, callbackTFMessage);
  ros::Subscriber subscriber_imu          = node.subscribe("/imu", 1, callbackIMU);
  ros::Subscriber subscriber_usound       = node.subscribe("/usound", 1, callbackRange);
  ros::Subscriber subscriber_laser        = node.subscribe("/laser", 1, callbackLaserScan);
  ros::Subscriber subscriber_camera_left  = node.subscribe("/camera_left", 1, callbackImage);
  ros::Subscriber subscriber_camera_right = node.subscribe("/camera_right", 1, callbackImage);

  //ds allocate an empty platform
  Platform platform;

  //ds target platform size (we will stop listening as soon as reached) TODO add other criteria?
  size_t target_platform_size = 4;
  bool platform_complete = false;

  //ds processing loop
  while (ros::ok()) {

    //ds trigger callbacks
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    //ds if we received messages
    if (!srrg_messages_to_process.empty()) {
      std::cerr << "received messages: " << srrg_messages_to_process.size() << std::endl;

      //ds process each message
      for (PropertyContainerSerializablePtr srrg_message: srrg_messages_to_process) {
        assert(srrg_message);

        //ds process message in platform - handling events and link/joint creation automatically (if necessary)
        platform.add(srrg_message);
        std::cerr << "current platform size: " << platform.size() << std::endl;
      }

      //ds check if platform is ready (according to its size here)
      if (!platform_complete && platform.size() >= target_platform_size) {
        std::cerr << "retrieved all information that was needed, setting up Platform" << std::endl;

        //ds check platform
        std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
        std::cerr << "calling Platform.setup()" << std::endl;
        platform.setup();
        std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
        std::cerr << platform << std::endl;
        platform_complete = true;
      }
      srrg_messages_to_process.clear();
    }
  }

  //ds done
  return ros::ok();
}
