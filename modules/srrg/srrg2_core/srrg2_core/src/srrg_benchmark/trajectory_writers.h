#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>

using TimestampIsometry3fMap =
  std::map<double,
           srrg2_core::Isometry3f,
           std::less<double>,
           Eigen::aligned_allocator<std::pair<double, srrg2_core::Isometry3f>>>;

void writeTrajectoryToFileTUM(const TimestampIsometry3fMap& conatiner_,
                              const std::string& filename_) {
  std::cerr << "writeTrajectoryToFileTUM|output file [" << FG_YELLOW(filename_) << "]" << std::endl;
  std::ofstream outfile(filename_, std::ofstream::out);
  if (!outfile.good() || !outfile.is_open()) {
    throw std::runtime_error("SLAMBenchmarkSuiteICL::writeTrajectoryToFile|unable to open file: " +
                             filename_);
  }
  outfile << std::fixed;
  outfile << std::setprecision(9);
  outfile << "# timestamp tx ty tz qx qy qz qw \n";
  for (const auto& entry : conatiner_) {
    const double& timestamp_seconds = entry.first;
    outfile << timestamp_seconds << " ";
    const srrg2_core::Isometry3f& pose_estimate(entry.second);
    outfile << pose_estimate.translation().x() << " ";
    outfile << pose_estimate.translation().y() << " ";
    outfile << pose_estimate.translation().z() << " ";
    const srrg2_core::Quaternionf orientation(pose_estimate.linear());
    outfile << orientation.x() << " ";
    outfile << orientation.y() << " ";
    outfile << orientation.z() << " ";
    outfile << orientation.w();
    outfile << std::endl;
  }
  outfile.close();
}

void writeTrajectoryToFileKITTI(const TimestampIsometry3fMap& conatiner_,
                                const std::string& filename_) {
  std::cerr << "writeTrajectoryToFileKITTI|output file [" << FG_YELLOW(filename_) << "]"
            << std::endl;
  std::ofstream outfile(filename_, std::ofstream::out);
  if (!outfile.good() || !outfile.is_open()) {
    throw std::runtime_error(
      "SLAMBenchmarkSuiteKITTI::writeTrajectoryToFile|unable to open file: " + filename_);
  }
  outfile << std::scientific;
  outfile << std::setprecision(9);
  for (const auto& entry : conatiner_) {
    const srrg2_core::Isometry3f& pose_estimate(entry.second);
    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 4; ++c) {
        if (r == 2 && c == 3) {
          outfile << pose_estimate.matrix()(r, c);
        } else {
          outfile << pose_estimate.matrix()(r, c) << " ";
        }
      }
    }
    outfile << std::endl;
  }
  outfile.close();
}
