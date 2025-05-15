#include <csignal>
#include <atomic>
// srrg SRRG
#include "srrg_messages_ros/instances.h"
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

using namespace srrg2_core;
using namespace srrg2_core_ros;

const std::string exe_name("srrg_to_ros_publisher_node");
const std::string default_src_name("source");
const std::string default_sink_name("sink");
#define LOG std::cerr << exe_name + "|"
std::atomic<bool> run{true};

static void sigIntHandler(int __attribute__((unused))) {
  std::cerr << "\n";
  LOG << FG_RED("termination request") << std::endl;
  run = false;
}

void generateConfig(const std::string& config_file_,
                    const std::string& config_name_source_,
                    const std::string& config_name_sink_) {
  ConfigurableManager manager;

  auto source = manager.create<MessageFileSource>(config_name_source_);
  auto sink   = manager.create<MessageROSSink>(config_name_sink_);
  manager.write(config_file_);

  LOG << "default config saved in [ " << config_file_ << " ]\n";
}

int main(int argc, char** argv) {
  // ia start ros
  ros::init(argc, argv, exe_name, ros::init_options::NoSigintHandler);
  srrgInit(argc, argv, exe_name.c_str());
  messages_ros_registerTypes();
  ParseCommandLine cmd_line(argv);

  // srrg check for options and params
  ArgumentFlag arg_generate_config(&cmd_line, "j", "generate-config", "generates a config file");
  ArgumentString arg_config_file(
    &cmd_line, "c", "config-file", "path to config file", "conf_" + exe_name + ".json");
  ArgumentString arg_config_name_source(
    &cmd_line, "ns", "name-src", "source name in the config file", default_src_name);
  ArgumentString arg_config_name_sink(
    &cmd_line, "ni", "name-sink", "sink name in the config file", default_sink_name);
  ArgumentString arg_input_file(&cmd_line, "i", "input", "boss input file", "");
  ArgumentInt arg_ros_rate(&cmd_line, "rr", "ros-rate", "ros rate to publish msgs (in ms)", 50);
  ArgumentInt arg_max_num_msgs(&cmd_line,
                               "mm",
                               "max-messages",
                               "maximum number of BOSS messages to read (default disabled)",
                               -1);
  ArgumentFlag arg_step_by_step(&cmd_line, "s", "step", "do step-by-step processing", false);
  cmd_line.parse();

  const std::string& config_file        = arg_config_file.value();
  const std::string& config_name_source = arg_config_name_source.value();
  const std::string& config_name_sink   = arg_config_name_sink.value();
  std::atomic<bool> step{arg_step_by_step.isSet()};

  // srrg check for options and params
  if (arg_generate_config.isSet()) {
    LOG << "generating default config\n";
    generateConfig(config_file, config_name_source, config_name_sink);
    return 0;
  }

  if (!arg_input_file.isSet()) {
    throw std::runtime_error(exe_name + "|ERROR, no input file specified");
  }

  const std::string& input_filename(arg_input_file.value());
  if (!srrg2_core::isAccessible(input_filename)) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access input file [ " + input_filename +
                             " ]");
  }

  // srrg Load configurations
  LOG << "loading config [ " << config_file << " ]\n";
  ConfigurableManager config_manager;
  config_manager.read(config_file);

  auto source = config_manager.getByName<MessageFileSource>(config_name_source);
  auto sink   = config_manager.getByName<MessageROSSink>(config_name_sink);
  if (source == nullptr || sink == nullptr) {
    throw std::runtime_error(exe_name + "|ERROR, wrong config");
  }

  // srrg open the tubes
  LOG << "opening input BOSS file [ " << FG_YELLOW(input_filename) << " ]\n";
  source->open(input_filename);
  sink->open();

  // ia override ros sigint catch
  signal(SIGINT, sigIntHandler);
  LOG << "starting the broadcasting @ [ " << arg_ros_rate.value() << " ] ms\n";

  int max_num_messages = -1;
  if (arg_max_num_msgs.isSet()) {
    LOG << "WARNING, processing only the first [ " << FG_YELLOW(arg_max_num_msgs.value())
        << " ] messages\n";
    max_num_messages = arg_max_num_msgs.value();
  }

  // srrg TODO handle the correct timing
  //     maybe use prev_time - curr_time, then put msg instead of sleep
  BaseSensorMessagePtr msg = nullptr;
  ros::Rate r(arg_ros_rate.value());
  int num_msgs_processed = 0;

  double cumulative_processing_time = 0;

  while (run && (msg = source->getMessage())) {
    SystemUsageCounter::tic();
    sink->putMessage(msg);
    cumulative_processing_time += SystemUsageCounter::toc();
    std::cerr << "\r" + exe_name << "|processed msg $ " << num_msgs_processed++;
    std::flush(std::cerr);

    if (max_num_messages > -1) {
      if (num_msgs_processed > max_num_messages) {
        break;
      }
    }
    r.sleep();
    if (step) {
      std::cerr << "\n" + exe_name + "|"
                << "press [ ENTER ] to continue\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      std::cin.get();
    }
  }
  std::cerr << "\n";
  LOG << "done" << std::endl;
  LOG << "processed [ " << num_msgs_processed << " ] msg in [ " << cumulative_processing_time
      << " ] s -- [ " << (float) (num_msgs_processed / (float) (cumulative_processing_time))
      << " ] Hz" << std::endl;

  LOG << "shutting down ... ";
  sink->close();
  source->close();
  std::cerr << "done!\n";

  return 0;
}
