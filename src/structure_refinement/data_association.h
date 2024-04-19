#pragma once

#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <memory>
#include "kdtree.cuh"

namespace structure_refinement {
using ContainerType = std::vector<Eigen::Vector3d>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType*;

struct SurfelMatches {
  TreeNodeTypePtr surfelA;
  TreeNodeTypePtr surfelB;
  // int tmp;
  bool matched;
};

class Surfelv2 {
  static uint64_t counter_;

 public:
  Surfelv2() : id_(counter_++){};
  ~Surfelv2() {};

  // Surfelv2(Surfelv2 & s): id_(counter_++){
  //   // id_ = s.id_;
  //   leafs_ = s.leafs_;
  //   meanEst_ = s.meanEst_;
  // }

  void setMeanEst(const Eigen::Vector3f & mean){
    meanEst_ = mean;
  }

  Eigen::Vector3f getMeanEst() const { return meanEst_; }
  Eigen::Vector3f getNormal() const { return leafs_[0]->eigenvectors_.col(0).cast<float>(); }

  static void resetTheSurfelsCounter(){
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

class DataAssociation {
public:
  DataAssociation() {}
  ~DataAssociation() {}

   void prepareData(std::vector<std::shared_ptr<TreeNodeType>> &, std::vector<std::vector<TreeNodeTypePtr>> &);
   void prepareDataCPU(std::vector<std::shared_ptr<TreeNodeType>> & kdTrees, std::vector<std::vector<TreeNodeTypePtr>> & kdTreeLeafes);
   void prepareDataExample();
   void processTheSurfelMatches(std::vector<SurfelMatches> &);
   std::vector<Surfelv2> & getSurfels() {return surfels_;}
   void resetSurfels() {
    surfels_.clear();
    Surfelv2::resetTheSurfelsCounter(); // Surfel's id is its position in a surfels_ vector, so it must be reset
  }
  std::vector<Surfelv2> surfels_;
};

}  // namespace structure_refinement
