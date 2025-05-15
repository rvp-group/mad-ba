#pragma once
#include "srrg_benchmark/slam_benchmark_suite.hpp"
#include "srrg_config/configurable_manager.h"
#include "srrg_messages_ros/instances.h"

namespace srrg2_core {

  // srrg 2D benchmark wrapper for simulation bags
  class SLAMBenchmarkSuiteSimul : public SLAMBenchmarkSuite<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using EstimateType = Isometry2f;

    virtual ~SLAMBenchmarkSuiteSimul() {
    }

    void loadDataset(const std::string& filepath_,
                     const size_t& number_of_message_packs_to_read_ = -1,
                     const size_t& number_of_message_pack_to_start_ = 0) override {
      _dataset_path                    = filepath_;
      _number_of_message_packs_to_read = number_of_message_packs_to_read_;
      _number_of_message_pack_to_start = number_of_message_pack_to_start_;

      // ds load dataset from disk
      srrg2_core_ros::MessageROSBagSourcePtr source(new srrg2_core_ros::MessageROSBagSource());

      source->param_topics.value().push_back("/diago_0/base_pose_ground_truth");

      source->open(filepath_);
      if (!source->isOpen()) {
        throw std::runtime_error("SLAMBenchmarkSuiteSimul::loadDataset|unable to load dataset: " +
                                 filepath_);
      }
      _relative_ground_truth_poses.clear();

      // ds relative estimate computation
      Isometry2f previous_pose(Isometry2f::Identity());

      // ds read file by tokens
      double timestamp = 0, previous_timestamp = std::numeric_limits<double>::max();

      // srrg create the gt
      while (auto msg = source->getMessage()) {
        if (auto odom_msg = std::dynamic_pointer_cast<OdometryMessage>(msg)) {
          const Isometry3f& pose = odom_msg->pose.value();
          timestamp              = odom_msg->timestamp.value();

          const Isometry2f pose_2d = geometry3d::get2dFrom3dPose(pose);

          if (timestamp - previous_timestamp > 0) {
            _relative_ground_truth_poses.push_back(RelativeEstimateStamped(
              pose_2d.inverse() * previous_pose, previous_timestamp, timestamp));
          }

          previous_timestamp = timestamp;
          previous_pose      = pose_2d;
        }
      }

      std::cerr << "SLAMBenchmarkSuiteSimul::loadDataset| gt size: "
                << _relative_ground_truth_poses.size() << std::endl;

      source->close();

      // srrg restart from the beginning
      source.reset(new srrg2_core_ros::MessageROSBagSource());
      source->open(filepath_);
      source->param_topics.value().clear();
      source->param_topics.value().push_back("/diago_0/scan");
      source->param_topics.value().push_back("/diago_0/odom");

      MessageSortedSourcePtr sorter(new MessageSortedSource());
      sorter->param_source.setValue(source);
      sorter->param_time_interval.setValue(0.1);

      _synchronizer = MessageSynchronizedSourcePtr(new MessageSynchronizedSource());
      _synchronizer->param_source.setValue(sorter);
      _synchronizer->param_topics.setValue(source->param_topics.value());
      _synchronizer->param_time_interval.setValue(0.1);
      std::cerr << "SLAMBenchmarkSuiteSimul::loadDataset|configured message playback for dataset: '"
                << filepath_ << "'" << std::endl;
      std::cerr << "SLAMBenchmarkSuiteSimul::loadDataset|topics: " << std::endl;
      for (const std::string& topic : _synchronizer->param_topics.value()) {
        std::cerr << topic << std::endl;
      }
    }

    void loadGroundTruth(const std::string& filepath_,
                         const std::string& filepath_additional_ = std::string()) override {
      throw std::runtime_error("SLAMBenchmarkSuiteSimul::loadGroundTruth|not implemented");
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
        throw std::runtime_error("SLAMBenchmarkSuiteSimul::getMessage|invalid message type: " +
                                 message->name());
      }

      auto laser_message = std::dynamic_pointer_cast<LaserMessage>(message_pack->messages[0]);
      auto odom_message  = std::dynamic_pointer_cast<OdometryMessage>(message_pack->messages[1]);
      if (!laser_message || !odom_message) {
        throw std::runtime_error("SLAMBenchmarkSuiteSimul::getMessage|wrong order of "
                                 "messages in pack check synchronizer configuration)");
      }

      // ds done
      return message_pack;
    }

    void reset() override {
      // ds TODO implement proper reset playback
      throw std::runtime_error("SLAMBenchmarkSuiteSimul::reset|ERROR: not implemented");
      _synchronizer->resetCounters();
    }

    void writeTrajectoryToFile(const std::string& filename_ = "trajectory.txt") const override {
      std::ofstream outfile(filename_, std::ofstream::out);
      if (!outfile.good() || !outfile.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteSimul::writeTrajectoryToFile|unable to open file: " + filename_);
      }
      outfile << std::fixed;
      outfile << std::setprecision(9);
      for (const auto& entry : _estimated_poses) {
        const double& timestamp_seconds = entry.timestamp_seconds;
        outfile << timestamp_seconds << " ";
        const EstimateType& pose_estimate(entry.estimate);
        outfile << pose_estimate.translation().x() << " ";
        outfile << pose_estimate.translation().y() << " ";
        const float& c = pose_estimate.linear()(0, 0);
        const float& s = pose_estimate.linear()(1, 0);
        outfile << atan2(s, c) << " ";
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
