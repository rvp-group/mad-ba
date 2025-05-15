#pragma once
#include "slam_benchmark_suite.hpp"
#include "srrg_config/configurable_manager.h"
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/image_message.h>

namespace srrg2_core {

  // ds 3D benchmark wrapper for: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html
  class SLAMBenchmarkSuiteICL : public SLAMBenchmarkSuite<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = Isometry3f;

    virtual ~SLAMBenchmarkSuiteICL() {
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
        throw std::runtime_error("SLAMBenchmarkSuiteICL::loadDataset|unable to load dataset: " +
                                 filepath_);
      }
      sorter->param_source.setValue(source);
      sorter->param_time_interval.setValue(0.01);

      // ds set up synchronizer for ICL datasets
      _synchronizer = MessageSynchronizedSourcePtr(new MessageSynchronizedSource());
      _synchronizer->param_source.setValue(sorter);
      _synchronizer->param_topics.value().push_back("/camera/rgb/image_color");
      _synchronizer->param_topics.value().push_back("/camera/depth/image");
      _synchronizer->param_topics.value().push_back("/camera/rgb/image_color/info");
      _synchronizer->param_topics.value().push_back("/camera/depth/image/info");
      _synchronizer->param_time_interval.setValue(0.01);
      std::cerr << "SLAMBenchmarkSuiteICL::loadDataset|configured message playback for dataset: '"
                << filepath_ << "'" << std::endl;
      std::cerr << "SLAMBenchmarkSuiteICL::loadDataset|topics: " << std::endl;
      for (const std::string& topic : _synchronizer->param_topics.value()) {
        std::cerr << topic << std::endl;
      }
    }

    void loadGroundTruth(const std::string& filepath_,
                         const std::string& filepath_additional_ = std::string()) override {
      _ground_truth_path = filepath_;

      // ds open ground truth file in TUM format: # timestamp tx ty tz qx qy qz qw
      std::ifstream ground_truth_file(filepath_, std::ios::in);
      if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteICL::loadGroundTruth|unable to read ground truth file: " + filepath_);
      }
      _relative_ground_truth_poses.clear();

      // ds relative estimate computation
      size_t previous_timestamp_seconds = 0;
      Isometry3f previous_pose(Isometry3f::Identity());

      // ds read file by tokens
      size_t timestamp_seconds = 0;
      double tx, ty, tz, qx, qy, qz, qw;
      while (ground_truth_file >> timestamp_seconds >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        // ds add measurement as relative estimate
        Isometry3f pose(Isometry3f::Identity());
        pose.translation() = Vector3f(tx, ty, tz);
        pose.linear()      = Matrix3f(Quaternionf(qw, qx, qz, qy));

        // ds fix first timestamp to zero (messages start at zero)
        assert(timestamp_seconds > 0);
        timestamp_seconds -= 1;

        // ds compute relative estimate to previous pose (except for the first measurement)
        if (previous_timestamp_seconds != 0 || timestamp_seconds != 0) {
          _relative_ground_truth_poses.push_back(RelativeEstimateStamped(
            pose.inverse() * previous_pose, previous_timestamp_seconds, timestamp_seconds));
        } else {
          // ds verify initial timestamp
          if (timestamp_seconds != 0) {
            throw std::runtime_error(
              "SLAMBenchmarkSuiteICL::loadGroundTruth|ERROR: unsupported ground truth format");
          }
        }
        previous_timestamp_seconds = timestamp_seconds;
        previous_pose              = pose;
      }

      std::cerr << "SLAMBenchmarkSuiteICL::loadGroundTruth|loaded relative poses: "
                << _relative_ground_truth_poses.size() << std::endl;
      ground_truth_file.close();
    }

    BaseSensorMessagePtr getMessage() override {
      BaseSensorMessagePtr message = _synchronizer->getMessage();
      ++_number_of_message_packs_read;

      // ds start gobbling up messages after the specified number (default 0, i.e. first message)
      while (_number_of_message_packs_read < _number_of_message_pack_to_start) {
        std::cerr << "SLAMBenchmarkSuiteICL::getMessage|skipping messsage: " << message->seq.value()
                  << std::endl;
        message = _synchronizer->getMessage();
        ++_number_of_message_packs_read;
      }

      // ds finished source or early termination
      if (!message || _number_of_message_packs_read - _number_of_message_pack_to_start >
                        _number_of_message_packs_to_read) {
        return nullptr;
      }

      // ds must be a message pack
      MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
      if (!message_pack) {
        throw std::runtime_error("SLAMBenchmarkSuiteICL::getMessage|invalid message type: " +
                                 message->name());
      }

      // ds load color and depth image from disk (buffered)
      // ds this is needed if we want to have buffered playback, otherwise the image links break
      ImageMessagePtr image_message_color =
        std::dynamic_pointer_cast<ImageMessage>(message_pack->messages[0]);
      ImageMessagePtr image_message_depth =
        std::dynamic_pointer_cast<ImageMessage>(message_pack->messages[1]);
      if (!image_message_color || !image_message_depth) {
        throw std::runtime_error("SLAMBenchmarkSuiteICL::getMessage|wrong order of "
                                 "messages in pack check synchronizer configuration)");
      }

      // ds done
      return message_pack;
    }

    void reset() override {
      // ds TODO implement proper reset playback
      throw std::runtime_error("SLAMBenchmarkSuiteICL::reset|ERROR: not implemented");
      _synchronizer->resetCounters();
    }

    void writeTrajectoryToFile(const std::string& filename_ = "trajectory.txt") const override {
      std::ofstream outfile(filename_, std::ofstream::out);
      if (!outfile.good() || !outfile.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteICL::writeTrajectoryToFile|unable to open file: " + filename_);
      }
      outfile << std::fixed;
      outfile << std::setprecision(9);
      for (const auto& entry : _estimated_poses) {
        const double& timestamp_seconds = entry.timestamp_seconds;
        outfile << timestamp_seconds << " ";
        const EstimateType& pose_estimate(entry.estimate);
        outfile << pose_estimate.translation().x() << " ";
        outfile << pose_estimate.translation().y() << " ";
        outfile << pose_estimate.translation().z() << " ";
        const Quaternionf orientation(pose_estimate.linear());
        outfile << orientation.x() << " ";
        outfile << orientation.y() << " ";
        outfile << orientation.z() << " ";
        outfile << orientation.w() << " ";
        outfile << std::endl;
      }
      outfile.clear();
    }

  protected:
    // ds we're loading messages on the spot as buffering image data consumes too much memory
    MessageSynchronizedSourcePtr _synchronizer;

    // ds live statistics/configuration
    size_t _number_of_message_packs_read    = 0;
    size_t _number_of_message_packs_to_read = 0;
    size_t _number_of_message_pack_to_start = 0;
  };

} // namespace srrg2_core
