#pragma once
#include "slam_benchmark_suite_icl.hpp"

namespace srrg2_core {

  // ds 3D benchmark wrapper for: https://vision.in.tum.de/data/datasets/rgbd-dataset
  class SLAMBenchmarkSuiteTUM : public SLAMBenchmarkSuiteICL {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = Isometry3f;

    virtual ~SLAMBenchmarkSuiteTUM() {
    }

    void loadDataset(const std::string& filepath_,
                     const size_t& number_of_message_packs_to_read_ = -1,
                     const size_t& number_of_message_pack_to_start_ = 0) override {
      _dataset_path                    = filepath_;
      _number_of_message_packs_to_read = number_of_message_packs_to_read_;
      _number_of_message_pack_to_start = number_of_message_pack_to_start_;

      // ds load dataset from disk
      MessageFileSourcePtr source(new MessageFileSource());
      MessageSortedSourcePtr sorter(new MessageSortedSource());
      source->open(filepath_);
      if (!source->isOpen()) {
        throw std::runtime_error("SLAMBenchmarkSuiteTUM::loadDataset|unable to load dataset: " +
                                 filepath_);
      }
      sorter->param_source.setValue(source);
      sorter->param_time_interval.setValue(0.03);

      // ds set up synchronizer for TUM datasets
      _synchronizer = MessageSynchronizedSourcePtr(new MessageSynchronizedSource());
      _synchronizer->param_source.setValue(sorter);
      _synchronizer->param_topics.value().push_back("/camera/rgb/image");
      _synchronizer->param_topics.value().push_back("/camera/depth/image");
      _synchronizer->param_topics.value().push_back("/camera/rgb/image/info");
      _synchronizer->param_topics.value().push_back("/camera/depth/image/info");
      _synchronizer->param_time_interval.setValue(0.03);
      std::cerr << "SLAMBenchmarkSuiteTUM::loadDataset|configured message playback for dataset: '"
                << filepath_ << "'" << std::endl;
      std::cerr << "SLAMBenchmarkSuiteTUM::loadDataset|topics: " << std::endl;
      for (const std::string& topic : _synchronizer->param_topics.value()) {
        std::cerr << topic << std::endl;
      }
    }

    // ds note that TUM is operating at milliseconds resolution for ground truth evaluation
    void loadGroundTruth(const std::string& filepath_,
                         const std::string& filepath_additional_ = std::string()) override {
      _ground_truth_path = filepath_;

      // ds open ground truth file in TUM format: # timestamp tx ty tz qx qy qz qw
      std::ifstream ground_truth_file(filepath_, std::ios::in);
      if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteTUM::loadGroundTruth|unable to read ground truth file: " + filepath_);
      }
      _relative_ground_truth_poses.clear();

      // ds relative estimate computation
      double previous_timestamp_seconds = 0;
      EstimateType previous_pose(EstimateType::Identity());

      // ds skip the first 3 lines (TUM format header information)
      std::cerr << "SLAMBenchmarkSuiteTUM::loadGroundTruth|header begin" << std::endl;
      std::string buffer;
      std::getline(ground_truth_file, buffer);
      std::cerr << buffer << std::endl;
      std::getline(ground_truth_file, buffer);
      std::cerr << buffer << std::endl;
      std::getline(ground_truth_file, buffer);
      std::cerr << buffer << std::endl;
      std::cerr << "SLAMBenchmarkSuiteTUM::loadGroundTruth|header end" << std::endl;

      // ds read file by tokens
      double timestamp_seconds = 0;
      double tx, ty, tz, qx, qy, qz, qw;
      while (ground_truth_file >> timestamp_seconds >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        // ds add measurement as relative estimate
        EstimateType pose(Isometry3f::Identity());
        pose.translation() = Vector3f(tx, ty, tz);
        pose.linear()      = Matrix3f(Quaternionf(qw, qx, qz, qy));
        assert(timestamp_seconds > 0);

        // ds compute relative estimate to previous pose (except for the first measurement)
        if (timestamp_seconds - previous_timestamp_seconds > 0) {
          _relative_ground_truth_poses.push_back(RelativeEstimateStamped(
            pose.inverse() * previous_pose, previous_timestamp_seconds, timestamp_seconds));
        }
        previous_timestamp_seconds = timestamp_seconds;
        previous_pose              = pose;
      }

      std::cerr << "SLAMBenchmarkSuiteTUM::loadGroundTruth|loaded relative poses: "
                << _relative_ground_truth_poses.size() << std::endl;
      ground_truth_file.close();
    }
  };

} // namespace srrg2_core
