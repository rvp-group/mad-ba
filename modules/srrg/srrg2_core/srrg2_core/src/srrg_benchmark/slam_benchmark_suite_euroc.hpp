#pragma once
#include "slam_benchmark_suite_kitti.hpp" //ds we can inherit most methods

namespace srrg2_core {

  // ds 3D benchmark wrapper for:
  // https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
  // ds this dataset contains stereo vision and IMU data in SE(3)
  class SLAMBenchmarkSuiteEuRoC : public SLAMBenchmarkSuiteKITTI {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = Isometry3f;

    virtual ~SLAMBenchmarkSuiteEuRoC() {
    }

    virtual void loadGroundTruth(const std::string& filepath_,
                                 const std::string& filepath_additional_ = std::string()) override {
      throw std::runtime_error(
        "SLAMBenchmarkSuiteEuRoC::loadGroundTruth|don't call my name, Alejandro");
    }

    virtual void loadDataset(const std::string& filepath_,
                             const size_t& number_of_message_packs_to_read_ = -1,
                             const size_t& number_of_message_pack_to_start_ = 0) override {
      throw std::runtime_error("SLAMBenchmarkSuiteEuRoC::loadDataset|don't call my name, Roberto");
    }

    void loadGroundTruthFromBOSS(const std::string& filepath_,
                                 const std::string& topic_ = "/ground_truth") override {
      _ground_truth_path = filepath_;

      MessageFileSourcePtr source(new MessageFileSource);
      MessageSortedSourcePtr sorter(new MessageSortedSource);

      source->open(filepath_);
      if (!source->isOpen()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteKITTI::loadGroundTruth|unable to load dataset: " + filepath_);
      }

      sorter->param_source.setValue(source);
      sorter->param_time_interval.setValue(0.1);

      // ds open ground truth file in EuRoC format
      std::ifstream ground_truth_stream(filepath_, std::ios::in);
      if (!ground_truth_stream.good() || !ground_truth_stream.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteEuRoC::loadGroundTruth|unable to read ground truth file: " +
          filepath_);
      }

      // ds skip the first line (EuRoC format header information)
      std::string buffer;
      std::getline(ground_truth_stream, buffer);

      // ds relative estimate computation
      _relative_ground_truth_poses.clear();
      double previous_timestamp_seconds          = 0;
      double previous_timestamp_absolute_seconds = 0;
      Isometry3f previous_pose(Isometry3f::Identity());

      // ds simultaneously populate absolute poses at 30 Hz (1.5x camera frequency)
      constexpr double minimum_timestamp_delta_for_absolute_poses = 1 / 30.0;
      _absolute_ground_truth_poses.reserve(10000);
      Isometry3f transform_shift_to_origin(Isometry3f::Identity());
      // ds read file by tokens
      double timestamp_seconds = 0;

      bool first_message           = true;
      BaseSensorMessagePtr message = nullptr;
      while ((message = sorter->getMessage())) {
        if (topic_ != message->topic.value()) {
          continue;
        }
        if (auto odom_ptr = std::dynamic_pointer_cast<OdometryMessage>(message)) {
          timestamp_seconds = odom_ptr->timestamp.value();

          Isometry3f pose = odom_ptr->pose.value();
          if (timestamp_seconds - previous_timestamp_seconds > 0) {
            // ds insertion must succeed otherwise there are duplicates!
            _relative_ground_truth_poses.push_back(RelativeEstimateStamped(
              pose.inverse() * previous_pose, previous_timestamp_seconds, timestamp_seconds));

            if (first_message) {
              _absolute_ground_truth_poses.emplace_back(
                AbsoluteEstimateStamped(Isometry3f::Identity(), timestamp_seconds));
              previous_timestamp_absolute_seconds = timestamp_seconds;
              transform_shift_to_origin           = pose.inverse();
              std::cerr
                << "SLAMBenchmarkSuiteEuRoC::loadGroundTruth|shifting absolute GT origin by: "
                << std::endl;
              std::cerr << transform_shift_to_origin.matrix() << std::endl;
            } else {
              // ds compute new absolute estimate if timestamp delta is sufficient
              if (timestamp_seconds - previous_timestamp_absolute_seconds >
                  minimum_timestamp_delta_for_absolute_poses) {
                _absolute_ground_truth_poses.emplace_back(
                  AbsoluteEstimateStamped(transform_shift_to_origin * pose, timestamp_seconds));
                previous_timestamp_absolute_seconds = timestamp_seconds;
              }
            }
          }
          previous_timestamp_seconds = timestamp_seconds;
          previous_pose              = pose;
        }
      }

      std::cerr << "SLAMBenchmarkSuiteEuRoC::loadGroundTruth|loaded relative poses: "
                << _relative_ground_truth_poses.size() << std::endl;
      std::cerr << "SLAMBenchmarkSuiteEuRoC::loadGroundTruth|loaded absolute poses: "
                << _absolute_ground_truth_poses.size() << " ("
                << 1 / minimum_timestamp_delta_for_absolute_poses << " Hz)" << std::endl;
    }

    void writeTrajectoryToFile(const std::string& filename_ = "trajectory.txt") const override {
      // ds write estimated trajectory to file (TUM format)
      _writeTrajectoryToFileTUM(filename_, _estimated_poses);
      std::cerr << "SLAMBenchmarkSuiteEuRoC::writeTrajectoryToFile|stored SLAM estimates in TUM "
                   "format: '"
                << filename_ << "' (messages: " << _estimated_poses.size() << ")" << std::endl;

      // ds create gt outfile (TUM format)
      _writeTrajectoryToFileTUM("gt.txt", _absolute_ground_truth_poses);
      std::cerr
        << "SLAMBenchmarkSuiteEuRoC::writeTrajectoryToFile|stored EuRoC GT in TUM format: 'gt.txt'"
        << " (messages: " << _absolute_ground_truth_poses.size() << ")" << std::endl;
    }

  protected:
    // ds TODO move helpers up and blast code
    void
    _writeTrajectoryToFileTUM(const std::string& filename_,
                              const AbsoluteEstimateStampedVector& poses_and_timestamps_) const {
      std::ofstream outfile(filename_, std::ofstream::out);
      if (!outfile.good() || !outfile.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteEuRoC::writeTrajectoryToFile|unable to open file: " + filename_);
      }
      outfile << std::fixed;
      outfile << std::setprecision(9);
      for (const auto& entry : poses_and_timestamps_) {
        outfile << entry.timestamp_seconds << " ";
        outfile << entry.estimate.translation().x() << " ";
        outfile << entry.estimate.translation().y() << " ";
        outfile << entry.estimate.translation().z() << " ";
        const Quaternionf orientation(entry.estimate.linear());
        outfile << orientation.x() << " ";
        outfile << orientation.y() << " ";
        outfile << orientation.z() << " ";
        outfile << orientation.w() << " ";
        outfile << std::endl;
      }
      outfile.close();
    }

    AbsoluteEstimateStampedVector _absolute_ground_truth_poses;
  };

} // namespace srrg2_core
