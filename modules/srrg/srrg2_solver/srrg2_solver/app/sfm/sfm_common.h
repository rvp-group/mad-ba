#pragma once
#include "srrg_geometry/epipolar.h"
#include <set>
#include <map>
#include <vector>
#include <iostream>
#include <memory>

using namespace srrg2_core;

namespace srrg2_solver {
  struct LandmarkMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int id;
    Eigen::Vector3f measurement;
    LandmarkMeasurement(int id_=-1,
                        const Eigen::Vector3f &meas_ = Eigen::Vector3f::Zero())
      : id(id_),
        measurement(meas_){}
  };

  using LandmarkMeasurementVector=std::vector<LandmarkMeasurement, Eigen::aligned_allocator<LandmarkMeasurement> >;
  
  struct Keyframe {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int id=-1;
    Eigen::Isometry3f gt_pose=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f est_pose=Eigen::Isometry3f::Identity();
    LandmarkMeasurementVector measurements;
    std::set<int> observed_landmark_ids;
    void finalize();
    void clear();
    inline bool empty() {return measurements.empty();}
  };
  
  std::ostream& operator<<(std::ostream& os , const Keyframe& kf);

  using KeyframePtr=std::shared_ptr<Keyframe>;
  using KeyframePtrMap=std::map<int, KeyframePtr>;

  void readKeyframes(KeyframePtrMap& keyframes, std::istream& is) ;

  struct CorrespondenceMatch {
    int from;
    int to;
    float error;
  };

  using CorrespondenceMatchVector = std::vector<CorrespondenceMatch>;

  struct EssentialPairwiseEstimate: public CorrespondenceMatch {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Isometry3f relative_estimate;
    int num_inliers;
    CorrespondenceMatchVector matches;
  };
  using EssentialPairwiseEstimatePtr = std::shared_ptr<EssentialPairwiseEstimate>;
  
  struct EssentialGraph {
    EssentialGraph(KeyframePtrMap& keyframes_)
      :_keyframes(keyframes_)
    {}

    using Partition=std::set<int>;
    using IdEstimateMap=std::multimap<int, EssentialPairwiseEstimatePtr>;
    void read(std::istream& is);
    void computePartitions(std::map<int, Partition>& partitions);
    IdEstimateMap _estimates;
    KeyframePtrMap& _keyframes;
  };
  
}
