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

  void addLeaf(TreeNodeTypePtr &leaf) {
    leaf->setSurfelId(this->id_);
    leafs_.push_back(leaf);
  }

  bool hasLeafFromPointCloud(int pointCloudIdx) {
    for (TreeNodeTypePtr leaf : leafs_) {
      if (leaf->pointcloud_id_ == pointCloudIdx)
        return true;
    }
    return false;
  }

//  protected:
  std::vector<TreeNodeTypePtr> leafs_;
  const uint64_t id_;
};

class DataAssociation {
public:
  DataAssociation() {}
  ~DataAssociation() {}

  __host__ void prepareData(std::vector<std::shared_ptr<TreeNodeType>> &, std::vector<std::vector<TreeNodeTypePtr>> &);
  __host__ void prepareDataExample();
  __host__ void processTheSurfelMatches(std::vector<SurfelMatches> &);
  __host__ std::vector<Surfelv2> & getSurfels() {return surfels_;}
  std::vector<Surfelv2> surfels_;

};

}  // namespace structure_refinement
