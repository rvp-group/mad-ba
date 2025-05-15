#include "srrg_benchmark/trajectory_writers.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;

const std::string exec_name = "extract_gt_from_srrg|";
#define LOG std::cerr << exec_name

void loadGroundTruthFromBOSS(TimestampIsometry3fMap& conatiner_,
                             const std::string& filepath_,
                             const std::string& topic_);

int main(int argc, char** argv) {
  srrg2_core::messages_registerTypes();

  std::string dataset_filename = "";

  ParseCommandLine cmd(argv);
  ArgumentString arg_dataset(
    &cmd, "i", "input", "input boss file with odom message as gt", "messages.json");
  ArgumentString arg_topic(&cmd, "t", "topic", "ground truth topic", "/ground_truth");
  ArgumentString arg_output(&cmd, "o", "output", "output file name", "gt.txt");
  ArgumentString arg_format(&cmd, "f", "format", "output format [tum, kitti]", "kitti");

  cmd.parse();
  dataset_filename = arg_dataset.value();
  if (!isAccessible(dataset_filename)) {
    throw std::runtime_error("cannot access [ " + dataset_filename + " ]");
  }

  TimestampIsometry3fMap gt_poses;
  loadGroundTruthFromBOSS(gt_poses, dataset_filename, arg_topic.value());

  const std::string& format = arg_format.value();

  if (format == "tum") {
    LOG << "writing gt in TUM format" << std::endl;
    writeTrajectoryToFileTUM(gt_poses, arg_output.value());
  } else if (format == "kitti") {
    LOG << "writing gt in KITTI format" << std::endl;
    writeTrajectoryToFileKITTI(gt_poses, arg_output.value());
  } else {
    throw std::runtime_error(exec_name + "|invalid output format [" + format +
                             "]. Available [tum, kitti]");
  }

  return 0;
}

void loadGroundTruthFromBOSS(TimestampIsometry3fMap& conatiner_,
                             const std::string& filepath_,
                             const std::string& topic_) {
  LOG << "opening file [" << FG_YELLOW(filepath_) << "] looking for topic [" << FG_BLUE(topic_)
      << "]" << std::endl;

  MessageFileSourcePtr source(new MessageFileSource);
  MessageSortedSourcePtr sorter(new MessageSortedSource);

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
    if (topic_ != message->topic.value()) {
      continue;
    }
    if (auto odom_ptr = std::dynamic_pointer_cast<OdometryMessage>(message)) {
      timestamp_seconds = odom_ptr->timestamp.value();
      Isometry3f pose   = odom_ptr->pose.value();
      conatiner_.insert(std::make_pair(timestamp_seconds, pose));
    }
  }

  LOG << "loaded gt poses: " << conatiner_.size() << std::endl;
}
