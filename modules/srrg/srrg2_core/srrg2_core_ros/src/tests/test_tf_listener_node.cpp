#include <ros/ros.h>
#include "srrg_data_structures/platform.h"
#include "srrg_converters/converter.h"
#include "srrg_property/property_container.h"
#include "srrg_property/property_serializable.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

//ds register requested SRRG message class
//BOSS_REGISTER_CLASS(PropertyTransformEventContainer)

//ds stack of global, generic messages in SRRG format, received from ROS
std::list<PropertyContainerSerializablePtr> srrg_messages_to_process;

//ds add to processing stack - perform SRRG message abstraction here
void callbackTFMessage(const tf2_msgs::TFMessageConstPtr& message_) {
  srrg_messages_to_process.push_back(Converter::convert(message_));
}

int main(int argc, char** argv) {

  //ds initialize roscpp
  ros::init(argc, argv, "test_tf_listener");

  //ds start node
  ros::NodeHandle node;
  srrg_messages_to_process.clear();

  //ds subscribe to TF topic
  ros::Subscriber subscriber_tf = node.subscribe("/tf", 1, callbackTFMessage);

  //ds allocate an empty platform
  Platform platform;

  //ds target platform size (we will stop listening as soon as reached) TODO add other criteria?
  size_t target_platform_size = 4;

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

        //ia process message in platform - handling events and link/joint creation automatically (if necessary)
        if (platform.add(srrg_message)) {
          std::cerr << "current platform size: " << platform.size() << std::endl;
        } else {
          std::cerr << "skipping message" << std::endl;
        }
      }

      //ds check if platform is ready (according to its size here)
      if (platform.size() >= target_platform_size) {
        std::cerr << "retrieved all information that was needed, setting up Platform" << std::endl;

        //ds check platform
        std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
        std::cerr << "calling Platform.setup()" << std::endl;
        platform.setup();
        std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
        std::cerr << platform << std::endl;
        break;
      }
      srrg_messages_to_process.clear();
    }
  }

  //ds done
  return ros::ok();
}
