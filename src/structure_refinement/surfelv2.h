#pragma once

#include <math.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

#include "kdtree.cuh"

namespace structure_refinement {
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

  Eigen::Vector3f getMeanEst() const { return meanEst_; }
  Eigen::Vector3f getNormalEst() const { return normalEst_; }

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
    Eigen::Vector3f meanAvg = Eigen::Vector3f::Zero();
    Eigen::Vector3f normalAvg = Eigen::Vector3f::Zero();
    for (uint i = 0; i < leafs_.size(); i++) {
      meanAvg += leafs_.at(i)->mean_;
      normalAvg += leafs_.at(i)->eigenvectors_.col(0);
    }
    meanAvg /= leafs_.size();
    normalAvg /= leafs_.size();
    normalAvg.normalize();
    // std::cout << "Mean  before " << meanEst_.transpose() << " Mean after: " << meanAvg.transpose() << std::endl;
    // std::cout << "Normal  before " << normalEst_.transpose() << " Normal after: " << normalAvg.transpose() << std::endl;
    meanEst_ = meanAvg;
    normalEst_ = normalAvg;
  }

  //  protected:
  std::vector<TreeNodeTypePtr> leafs_;
  uint64_t id_;
  Eigen::Vector3f meanEst_;
  Eigen::Vector3f normalEst_;
};

}  // namespace structure_refinement
