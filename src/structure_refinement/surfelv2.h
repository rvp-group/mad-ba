#pragma once

#include <math.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

#include "kdtree.cuh"

namespace structure_refinement {
using ContainerType = std::vector<Eigen::Vector3d>;
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

  Eigen::Vector3f getMeanEst() const { return meanEst_; }
  Eigen::Vector3f getNormal() const { return leafs_[0]->eigenvectors_.col(0).cast<float>(); }

  static void resetTheSurfelsCounter() {
    counter_ = 0;
  }

  void addLeaf(TreeNodeTypePtr &leaf) {
    leaf->setSurfelId(this->id_);
    leafs_.push_back(leaf);
    if (leafs_.size() == 1)
      meanEst_ = leaf->mean_.cast<float>();
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

  //  protected:
  std::vector<TreeNodeTypePtr> leafs_;
  const uint64_t id_;
  Eigen::Vector3f meanEst_;
};

}  // namespace structure_refinement
