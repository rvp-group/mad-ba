#include <csignal>
//srrg SRRG

#include <srrg_system_utils/system_utils.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include "srrg_messages_ros/instances.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

const char* banner[] = {
  "",
  "srrg_to_rosbag_converter_app: converts a srrg_boss file into a rosbag",
  "",
  "usage:",
  " ./srrg_to_rosbag_converter_app [options] <boss_file>",
  " where [options]: ",
  0

};

volatile bool run = true;

static void sigIntHandler(int __attribute__((unused))) {
  std::cerr << "Got user interrupt, Terimnating" << std::endl;
  run = false;
}

void generateConfig(const std::string& config_file_, const std::string& conf_name_) {
  ConfigurableManager manager;
 
  MessageFileSourcePtr source = manager.create<MessageFileSource>(conf_name_);

  manager.write(config_file_);
}

int main(int argc, char** argv) {

  if (argc < 2) {
    srrg2_core::printBanner(banner);
    return 0;
  }

  messages_ros_registerTypes();
  ParseCommandLine cmd_line(argv, banner);

  std::string input_boss_file;              // in.boss
  std::string output_rosbag_file = "out.bag"; // out.bag
  std::string config_name = "srrg_to_rosbag_converter_app";
  std::string config_file = config_name + ".conf";

  //srrg check for options and params
  ArgumentFlag arg_generate_config(&cmd_line, "j", "generate-config", "generates a config file");
  ArgumentString arg_config_file(&cmd_line, "c", "config-file", "path to config file", config_file);
  ArgumentString arg_config_name(&cmd_line, "cn", "config-name", "name of the configuration", config_name);
  ArgumentString arg_output_file(&cmd_line, "o", "output", "output bag file", output_rosbag_file);

  cmd_line.parse();

  config_file = arg_config_file.value();
  config_name = arg_config_name.value();
  output_rosbag_file = arg_output_file.value();

  //srrg check for options and params
  if (arg_generate_config.isSet()) {
    generateConfig(config_file, config_name);
    return 0;
  }

  if (!output_rosbag_file.length()) {
    std::cerr << "no output filename specified, exiting" << std::endl;
    return 0;
  }

  if (cmd_line.lastParsedArgs().empty()){
    std::cerr << "no input file specified, exiting" << std::endl;
    return 0;
  }
  input_boss_file=cmd_line.lastParsedArgs()[0];
  
  std::cerr << "output_file: " << output_rosbag_file << std::endl;
  std::cerr << "rosbag_file: " << input_boss_file << std::endl;

  ConfigurableManager config_manager;
  config_manager.read(config_file);
  std::cerr << "reading config from [" << config_file << "]" << std::endl;

  MessageFileSourcePtr source = config_manager.getByName<MessageFileSource>(config_name);
  if (!source) {
    std::cerr << "cast fail" << std::endl;
    return -1;
  }

  signal(SIGINT, sigIntHandler);

  source->open(input_boss_file);


  //srrg open the sink
  srrg2_core_ros::MessageROSBagSink sink;
  sink.open(output_rosbag_file);
  BaseSensorMessagePtr msg;

  SystemUsageCounter usage_counter;
  int num_msgs_processed = 0;

  double processing_time = 0;

  while (run && (msg = source->getMessage())) {
    usage_counter.tic();
    sink.putMessage(msg);
    std::cerr << "msg: " << num_msgs_processed++ << "\r";
    processing_time += usage_counter.toc();
  }

  sink.close();

  std::cerr << std::endl;

  config_manager.write(config_file);
  std::cerr << "Done" << std::endl;
  std::cerr << "Processed " << num_msgs_processed
            << " msg in " << processing_time
            << "sec (" << num_msgs_processed / processing_time << " frames/sec)" << std::endl;
  std::cerr << "Avg Memory: " << usage_counter.totalMemory() / 1024 << " Kb" << std::endl;

  return 0;

}
