#include <csignal>
//srrg SRRG
#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_config/configurable_manager.h>
#include "srrg_messages_ros/instances.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

const char* banner[] = {
  "",
  "ros_to_srrg_listener_node: publish srrg_messages on ros topics",
  "",
  "usage:",
  " ./ros_to_srrg_listener_node [options]",
  " where [options]: ",
  0

};

volatile bool run = true;

static void sigIntHandler(int __attribute__((unused))) {
  std::cerr << "Got user interrupt, Terimnating" << std::endl;
  run = false;
}

void generateConfig(const std::string& config_file_,
                    const std::string& config_name_source_,
                    const std::string& config_name_sink_) {
  ConfigurableManager manager;
  auto source = manager.create<MessageROSSource>(config_name_source_);
  auto sink   = manager.create<MessageFileSink>(config_name_sink_);  
  sink->setName(config_name_sink_);
  manager.write(config_file_);
}

int main(int argc, char** argv) {

  std::string config_name = "ros_to_srrg_listener_node";
  std::string config_file = config_name + ".conf";
  std::string config_name_source = config_name + "_source";
  std::string config_name_sink = config_name + "_sink";

  std::string output_boss_file = "out.boss";

  ros::init(argc, argv, config_name);
  srrgInit(argc, argv);
  
  messages_ros_registerTypes();
  ParseCommandLine cmd_line(argv, banner);

  //srrg check for options and params
  ArgumentFlag arg_generate_config(&cmd_line, "j", "generate-config", "generates a config file");
  ArgumentString arg_output_file(&cmd_line, "o", "output", "output boss file", output_boss_file);
  ArgumentString arg_config_file(&cmd_line, "c", "config-file", "path to config file", config_file);
  ArgumentString arg_config_name_source(&cmd_line, "sr", "config-source", "name of the configuration", config_name_source);
  ArgumentString arg_config_name_sink(&cmd_line, "sk", "config-sink", "name of the configuration", config_name_sink);
  cmd_line.parse();

  config_file = arg_config_file.value();
  config_name_source = arg_config_name_source.value();
  config_name_sink = arg_config_name_sink.value();
  output_boss_file = arg_output_file.value();

  //srrg check for options and params
  if (arg_generate_config.isSet()) {
    generateConfig(config_file, config_name_source, config_name_sink);
    std::cerr << "[ros_to_srrg_listener_node]| Generated configuration "
              << BGREEN << config_file << RESET << std::endl;
    return 0;
  }

  if (!output_boss_file.length()) {
    std::cerr << "no output filename specified" << std::endl;
    return 0;
  }

  std::cerr << "output_file: " << output_boss_file << std::endl;

  signal(SIGINT, sigIntHandler);
  
  
  //srrg Load configurations
  ConfigurableManager config_manager;
  config_manager.read(config_file);
  std::cerr << "reading config from [" << config_file << "]" << std::endl;
  std::shared_ptr<MessageROSSource> source = config_manager.getByName<MessageROSSource>(config_name_source);
  if (!source) {
    std::cerr << "source cast fail" << std::endl;
    return -1;
  }
  std::shared_ptr<MessageFileSink> sink = config_manager.getByName<MessageFileSink>(config_name_sink);
  if (!sink) {
    std::cerr << "sink cast fail" << std::endl;
    return -1;
  }

  //srrg open the tubes
  source->open();
  if (!source->isOpen()) {
    std::cerr << "timeout... exiting" << std::endl;
    return -1;
  }
  std::cerr << "source is open...";

  sink->open(output_boss_file);
  std::cerr << " sink is open." << std::endl;

  SystemUsageCounter usage_counter;
  int num_msgs_processed = 0;

  double processing_time = 0;

  while (source->ok()) {
    MessagePackPtr pack = std::dynamic_pointer_cast<MessagePack>(source->getMessage());
    if (!pack) {
      continue;
    }
    for (auto message_in_pack: pack->messages) {
      if (BaseSensorMessagePtr inner_msg = std::dynamic_pointer_cast<BaseSensorMessage>(message_in_pack)) {
        usage_counter.tic();
        sink->putMessage(inner_msg);
        std::cerr << "msg: " << num_msgs_processed++ << "\r";
        processing_time += usage_counter.toc();
      }
    }
  }
  std::cerr << std::endl;

  config_manager.write(config_file);
  std::cerr << "Done" << std::endl;
  std::cerr << "Processed " << num_msgs_processed
            << " msg in " << processing_time
            << "sec (" << num_msgs_processed / processing_time << " frames/sec)" << std::endl;
  std::cerr << "Avg Memory: " << usage_counter.totalMemory() / 1024 << " Kb" << std::endl;

  sink->close();
  std::cerr << "sink closed... ";
  source->close();
  std::cerr << "source closed." << std::endl;

  std::cerr << "Done" << std::endl;

  return 0;
}
