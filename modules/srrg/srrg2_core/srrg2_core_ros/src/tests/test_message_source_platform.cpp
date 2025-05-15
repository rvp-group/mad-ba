#include <iostream>
#include <thread>
#include <signal.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/message_handlers/message_source_platform.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/parse_command_line.h>

#include "srrg_messages_ros/instances.h"

const std::string exe_prefix = "test_message_source_platform|";
#define LOG std::cerr << exe_prefix

bool run = true;
void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    LOG << "shutting down...PRESS ENTER\n";
    run = false;
  }
}

const char* banner[] = {
    "usage: <exe> [options] path_to_bag.bag",
    "test for message source platform, to shit transform tree",
    0
};


using namespace srrg2_core;
using namespace srrg2_core_ros;

int main(int argc, char **argv) {
  signal(SIGINT, sigIntHandler);

  messages_registerTypes();
  messages_ros_registerTypes();

  ParseCommandLine cmd_line(argv, banner);
  ArgumentString topic_tf(&cmd_line, "tt", "topic-tf", "tf topic", "/tf");
  ArgumentFlag stepwise_parsing(&cmd_line, "step", "stepwise-processing", "process the bag step by step (press ENTER to go forward)");
  cmd_line.parse();

  if (cmd_line.lastParsedArgs().empty())
    throw std::runtime_error(exe_prefix+"please set valid bag");

  MessageROSBagSourcePtr source = MessageROSBagSourcePtr(new MessageROSBagSource);
  source->param_topics.value().push_back(topic_tf.value());
  source->param_topics.value().push_back("/scan");

  MessageSortedSourcePtr sorter = MessageSortedSourcePtr(new MessageSortedSource);
  sorter->param_time_interval.setValue(1.0);
  //ia setup platform_source: source, tf_topics name (each one corresponds to separate tree)
  MessageSourcePlatformPtr transform_tree_source = MessageSourcePlatformPtr(new MessageSourcePlatform);
  transform_tree_source->param_source.setValue(source);
  transform_tree_source->param_tf_topics.value().push_back(topic_tf.value());
  //ia setup platform_source: bind the goddamn topics to the transform tree
  transform_tree_source->bindTfTopics();

  sorter->param_source.setValue(transform_tree_source);
  //ia open file
  LOG << "opening file " << FG_YELLOW(cmd_line.lastParsedArgs()[0]) << std::endl;
  source->open(cmd_line.lastParsedArgs()[0]);
  
  while (run) {

    //ia ask for a message. if it is a transform message it will
    //ia be added to the transform tree and then returned.
    //ia everything else is just returned as it is.
    BaseSensorMessagePtr msg = transform_tree_source->getMessage();
    if (msg == 0) {
      LOG << "source ended\n";
      run = false;
      break;
    }
    LOG << "message timestamp = " << std::setprecision(20) << FG_CYAN(msg->timestamp.value()) << std::endl;

    PlatformPtr p = transform_tree_source->platform(topic_tf.value());
    if (!p) throw std::runtime_error(exe_prefix+"this shouldn't happen");
    /* LOG << *p << std::endl; */
    //ia ask for transforms.
    Isometry3f transform;
    /* if(p->getTransform(transform,"odom","base_link", msg->timestamp.value())){ */
    /*   LOG << FG_BLUE("odom in base_link transform:\n") */
    /*       << transform.matrix() << std::endl; */
    /* } */
    LaserMessagePtr l = std::dynamic_pointer_cast<LaserMessage> (msg);
    if(l) {
      /* if(p->getTransform(transform,l->frame_id.value(),"base_link")){ */
      /*   LOG << FG_BLUE("laser message frame id in base_link: \n") */
      /*       << transform.matrix() << std::endl; */
      /* } */
      /* if(p->getTransform(transform,"odom", l->frame_id.value(),msg->timestamp.value())){ */
      /*   LOG << FG_BLUE("odom in laser frame: \n") */
      /*       << transform.matrix() << std::endl; */
      /* } */
      /* if(p->getTransform(transform,l->frame_id.value(), msg->timestamp.value())){ */
      /*   LOG << FG_BLUE("laser message frame id : \n") */
      /*       << transform.matrix() << std::endl; */
      /* } */
      LOG << msg->timestamp.value() << std::endl;
      if(p->getTransform(transform,l->frame_id.value(),msg->timestamp.value())){
        LOG << FG_BLUE("laser message frame id : \n")
            << transform.matrix() << std::endl;
      }
      if(p->getTransform(transform,l->frame_id.value())){
        LOG << FG_BLUE("laser message frame id : \n")
            << transform.matrix() << std::endl;
      }
    }
    if (stepwise_parsing.isSet()) {
      LOG << "press ENTER to continue" << std::endl;
      std::cin.clear();
      std::cin.get();
    }
  }

  LOG << "closing\n";
  return 0;
}




