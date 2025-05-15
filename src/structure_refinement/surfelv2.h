#pragma once

#include <math.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

#include "kdtree.cuh"

namespace mad_ba {
using ContainerType = std::vector<Eigen::Vector3f>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType *;

class Surfelv2 {
  static uint64_t counter_;

 public:
  Surfelv2() : id_(counter_++){};
  ~Surfelv2(){};

  void setMeanEst(const Eigen::Vector3f &mean) {
    meanEst_ = mean;
  }

  void setNormalEst(const Eigen::Vector3f &normal) {
    normalEst_ = normal;
  }

  const Eigen::Vector3f & getMeanEst() const { return meanEst_; }
  const Eigen::Vector3f & getNormalEst() const { return normalEst_; }

  static void resetTheSurfelsCounter() {
    counter_ = 0;
  }

  void addLeaf(TreeNodeTypePtr &leaf) {
    leaf->setSurfelId(this->id_);
    leafs_.push_back(leaf);
    if (leafs_.size() == 1){
      meanEst_ = leaf->mean_.cast<float>();
      normalEst_ = leaf->eigenvectors_.col(0);
    }
    else
      averageMeanAndNormal();
  }

  bool hasLeafFromPointCloud(int pointCloudIdx) {
    for (TreeNodeTypePtr leaf : leafs_) {
      if (leaf->pointcloud_id_ == pointCloudIdx)
        return true;
    }
    return false;
  }

  float getMaxRadius() const {
    float maxRadius = 0;
    for (TreeNodeTypePtr leaf : leafs_) {
      float r = sqrt(leaf->bbox_[1] * leaf->bbox_[1] + leaf->bbox_[2] * leaf->bbox_[2]) / 2.0;
      if (r > maxRadius)
        maxRadius = r;
    }
    return maxRadius;
  }

  void averageMeanAndNormal() {
    Eigen::Vector3d meanAvg = Eigen::Vector3d::Zero();
    Eigen::Vector3d normalAvg = Eigen::Vector3d::Zero();
    for (uint i = 0; i < leafs_.size(); i++) {
      meanAvg += leafs_.at(i)->mean_.cast<double>();
      normalAvg += leafs_.at(i)->eigenvectors_.col(0).cast<double>();
    }
    meanAvg /= leafs_.size();
    normalAvg /= leafs_.size();
    normalAvg.normalize();
    // std::cout << "Mean  before " << meanEst_.transpose() << " Mean after: " << meanAvg.transpose() << std::endl;
    // std::cout << "Normal  before " << normalEst_.transpose() << " Normal after: " << normalAvg.transpose() << std::endl;
    meanEst_ = meanAvg.cast<float>();
    normalEst_ = normalAvg.cast<float>();
  }

  //  protected:
  std::vector<TreeNodeTypePtr> leafs_;
  uint64_t id_;
  Eigen::Vector3f meanEst_;
  Eigen::Vector3f normalEst_;
};

}  // namespace mad_ba
