#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "kdtree.cuh"
#include "surfelv2.h"
#include "mad_ba/surfel_matches.h"

namespace mad_ba {
using ContainerType = std::vector<Eigen::Vector3f>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType *;

class DataAssociation {
 public:
  DataAssociation(float maxDst, float maxDstNorm, float maxAngle) : maxDst_(maxDst), maxDstNorm_(maxDstNorm), maxAngle_(maxAngle) {
  }
  ~DataAssociation() {}

  void associateDataKernelCPU(int, int, int, TreeNodeTypePtr *, TreeNodeTypePtr *, SurfelMatches *);
  void prepareDataCPU(std::vector<std::unique_ptr<TreeNodeType>> &kdTrees, std::vector<std::vector<TreeNodeTypePtr>> &kdTreeLeafes);
  void processTheSurfelMatches(std::vector<SurfelMatches> &);
  void filterSurfels();
  void resetSurfels();
  void decimateSurfels(int);
  std::vector<Surfelv2> &getSurfels() { return surfels_; }

 private:
  std::vector<Surfelv2> surfels_;
  float maxDst_;
  float maxDstNorm_;
  float maxAngle_;
};

}  // namespace mad_ba
