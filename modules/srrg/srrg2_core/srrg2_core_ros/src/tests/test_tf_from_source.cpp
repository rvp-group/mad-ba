#include <Eigen/StdVector>
#include <iostream>
#include <thread>

#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/parse_command_line.h>

#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/instances.h>

#include "srrg_messages_ros/instances.h"
#include "srrg_data_structures/platform.h"

#include <signal.h>

const std::string exe_prefix = "test_tf_from_sync|";
#define LOG std::cerr << exe_prefix

using namespace srrg2_core;
using namespace srrg2_core_ros;

const char* banner[] = {
    "test to read tf from a sync source",
    "usage: <exe> [options] path_to_bag.bag",
    "BAG MUST CONTAIN: laser, odometry and tf",
    0
};

bool run = true;
void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    LOG << "shutting down...PRESS ENTER\n";
    run = false;
  }
}

using Isometry3fAlignedVector = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f> >;

class Evil {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Evil() {}
  ~Evil() {}

  inline Platform& getPlatform() {return _platform;}

  void addEvil(PropertyContainerIdentifiablePtr& events_ptr) {
    std::cerr << "Evil::addEvil|adding event vector\n";
    _platform.add(events_ptr);
    if (!_platform.isWellFormed()) {
      _platform.setup();
    }

    std::cerr << "Evil::addEvil|pushing back dummy stuff in the vector\n";
    std::cerr << "Evil::addEvil|size PRE = " << _poses.size();
    _poses.push_back(Isometry3f::Identity());
    std::cerr << "\tsize POST = " << _poses.size() << std::endl;
  }

protected:
  Platform _platform;
  Isometry3fAlignedVector _poses;
};


int main(int argc, char **argv) {
  signal(SIGINT, sigIntHandler);

  messages_registerTypes();
  messages_ros_registerTypes();

  ParseCommandLine cmd_line(argv, banner);
  ArgumentString topic_laser(&cmd_line, "tl", "topic-laser", "laser topic", "/scan");
  ArgumentString topic_odom(&cmd_line, "to", "topic-odom", "odometry topic", "/odom");
  ArgumentString topic_tf(&cmd_line, "tt", "topic-tf", "tf topic", "/tf");
  ArgumentFlag stepwise_parsing(&cmd_line, "step", "stepwise-processing", "process the bag step by step (press ENTER to go forward)");
  cmd_line.parse();

  if (cmd_line.lastParsedArgs().empty())
    throw std::runtime_error(exe_prefix+"please set valid bag");

  MessageROSBagSourcePtr source = MessageROSBagSourcePtr(new MessageROSBagSource);
  MessageSortedSourcePtr sorter = MessageSortedSourcePtr(new MessageSortedSource);
  sorter->param_time_interval.setValue(100.0);
  //ia if I comment this block and I use the synched source (so this is not relevant)
  //ia then everything goes nuts iff Platform is a pointer.
  //ia otherwise (when Platform is on the stack) everything is ok.

  source->param_topics.value().push_back(topic_laser.value());
  source->param_topics.value().push_back(topic_odom.value());
  source->param_topics.value().push_back(topic_tf.value());
  sorter->param_source.setValue(source);

  LOG << "opening file " << FG_YELLOW(cmd_line.lastParsedArgs()[0]) << std::endl;
  source->open(cmd_line.lastParsedArgs()[0]);


  Platform platform;
  BaseSensorMessagePtr msg = nullptr;

  size_t m = 0;
  std::cerr << std::setprecision(20) << std::endl;
  Isometry3f T;
  while (run) {
    msg = sorter->getMessage();
    if (!msg) {
      LOG << "file ended, stop" << std::endl;
      break;
    }

    LOG << "message #" << FG_CYAN(m++) << std::endl;
    double timestamp = 0;
    if (!platform.add(msg)) {
      LOG << "failed to add the message to the platform" << std::endl;
      continue;
    }

    timestamp = msg->timestamp.value();

    //ia print
    LOG << "tf message timestamp: " << timestamp << std::endl;
    LOG << "platform size = " << FG_YELLOW(platform.size()) << std::endl;
    /* if(platform.getTransform(T,"laser_frame", "base_link", timestamp)) */
    /*   LOG << "transform laser_frame base_link = \n" << T.matrix() << std::endl; */
    if(platform.getTransform(T,"base_link","odom",timestamp))
      LOG << "transform base_link in odom = \n" << T.matrix() << std::endl;
    LOG << platform << std::endl;
    if (stepwise_parsing.isSet()) {
      LOG << "press ENTER to continue" << std::endl;
      std::cin.clear();
      std::cin.get();
    }

    std::cerr << "==================================================================================================\n\n";
  }

//  delete platform;
}



