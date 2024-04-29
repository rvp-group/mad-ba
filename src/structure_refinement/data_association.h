#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "kdtree.cuh"
#include "surfelv2.h"
#include "structure_refinement/surfel_matches.h"

namespace structure_refinement {
using ContainerType = std::vector<Eigen::Vector3f>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType *;

class DataAssociation {
 public:
  DataAssociation() {}
  ~DataAssociation() {}

  void associateDataKernelCPU(int, int, int, TreeNodeTypePtr *, TreeNodeTypePtr *, SurfelMatches *);
  void prepareDataCPU(std::vector<std::unique_ptr<TreeNodeType>> &kdTrees, std::vector<std::vector<TreeNodeTypePtr>> &kdTreeLeafes);
  void processTheSurfelMatches(std::vector<SurfelMatches> &);
  void resetSurfels();
  std::vector<Surfelv2> &getSurfels() { return surfels_; }

 private:
  std::vector<Surfelv2> surfels_;
};

}  // namespace structure_refinement
