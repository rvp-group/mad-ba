#include "srrg_messages_ros/instances.h"
#include <srrg_benchmark/trajectory_writers.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

using namespace srrg2_core;
using namespace srrg2_core_ros;

const std::string exe_name = "extract_gt_from_bag";
#define LOG std::cerr << exe_name << "|"

void loadGroundTruthFromROSBAG(TimestampIsometry3fMap& conatiner_,
                               const std::string& filepath_,
                               const std::string& topic_);

int main(int argc, char** argv) {
  srrgInit(argc, argv, exe_name.c_str());

  messages_ros_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_dataset(&cmd, "i", "input", "input boss file with odom message as gt", "");
  ArgumentString arg_topic(&cmd, "t", "topic", "ground truth topic", "/ground_truth");
  ArgumentString arg_output(&cmd, "o", "output", "output file name", "gt.txt");
  ArgumentString arg_format(&cmd, "f", "format", "output format [tum, kitti]", "kitti");

  cmd.parse();
  if (!arg_dataset.isSet() || !arg_topic.isSet()) {
    LOG << FG_RED("ERROR, not enough arguments\nrun with --help option");
    throw std::runtime_error(exe_name + "|ERROR, invalid shell arguments");
  }

  std::string dataset_filename(std::move(arg_dataset.value()));
  std::string topic(std::move(arg_topic.value()));

  if (!isAccessible(dataset_filename)) {
    throw std::runtime_error("cannot access [ " + dataset_filename + " ]");
  }

  TimestampIsometry3fMap gt_poses;
  loadGroundTruthFromROSBAG(gt_poses, dataset_filename, topic);

  const std::string format(std::move(arg_format.value()));

  if (format == "tum") {
    LOG << "writing gt in TUM format" << std::endl;
    writeTrajectoryToFileTUM(gt_poses, arg_output.value());
  } else if (format == "kitti") {
    LOG << "writing gt in KITTI format" << std::endl;
    writeTrajectoryToFileKITTI(gt_poses, arg_output.value());
  } else {
    throw std::runtime_error(exe_name + "|invalid output format [" + format +
                             "]. Available [tum, kitti]");
  }

  return 0;
}

void loadGroundTruthFromROSBAG(TimestampIsometry3fMap& conatiner_,
                               const std::string& filepath_,
                               const std::string& topic_) {
  LOG << "opening file [" << FG_YELLOW(filepath_) << "] looking for topic [" << FG_BLUE(topic_)
      << "]" << std::endl;

  MessageROSBagSourcePtr source(new MessageROSBagSource);
  MessageSortedSourcePtr sorter(new MessageSortedSource);
  source->param_topics.value() = {topic_};
  source->open(filepath_);
  if (!source->isOpen()) {
    throw std::runtime_error("SLAMBenchmarkSuiteKITTI::loadGroundTruth|unable to load dataset: " +
                             filepath_);
  }

  sorter->param_source.setValue(source);
  sorter->param_time_interval.setValue(0.1);

  conatiner_.clear();
  // ds read file by tokens
  double timestamp_seconds = 0;

  BaseSensorMessagePtr message = nullptr;
  while ((message = sorter->getMessage())) {
    if (auto odom_ptr = std::dynamic_pointer_cast<OdometryMessage>(message)) {
      timestamp_seconds = odom_ptr->timestamp.value();
      Isometry3f pose   = odom_ptr->pose.value();
      conatiner_.insert(std::make_pair(timestamp_seconds, pose));
    } else {
      LOG << FG_RED("unhandled message type [") << FG_YELLOW(message->className())
          << FG_RED("] for topic [") << FG_YELLOW(message->topic.value()) << "]" << std::endl;
    }
  }

  LOG << "loaded gt poses: " << conatiner_.size() << std::endl;
}
