#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "srrg_data_structures/platform.h"
#include "srrg_converters/converter.h"
#include "srrg_property/property_container.h"
#include "srrg_property/property_serializable.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

//ds register requested SRRG message class
//BOSS_REGISTER_CLASS(PropertyTransformEventContainer)




int main(int argc, char** argv) {
  if (argc < 2)
    throw std::runtime_error("test_tf_listener_playback|please specify bag file as argument");

  const std::string bagfile(argv[1]);

  Platform platform;
  rosbag::Bag bag(bagfile);
  rosbag::View view(bag);

  //ds process each message in the rosbag
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    std::cerr << "processing ROS message at time (s): " << it->getTime() << std::endl;

    //ds if we're interested in this message
    if (it->getTopic() == "/tf") {

      //ds convert into generic SRRG message
      PropertyContainerSerializablePtr srrg_message = Converter::convert<tf2_msgs::TFMessage>(*it);

      //ia process message in platform - handling events and link/joint creation automatically (if necessary)
      if (!platform.add(srrg_message)) {
        std::cerr << "skipping message" << std::endl;
      }

      //ds process message also with other components
      //...
    }
    std::cerr << "current platform size: " << platform.size() << std::endl;
  }

  //ds check platform
  std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
  std::cerr << "calling Platform.setup()" << std::endl;
  platform.setup();
  std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
  std::cerr << platform << std::endl;
  return 0;
}
