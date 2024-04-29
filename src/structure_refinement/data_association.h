#pragma once

#include <math.h>
#include <omp.h>
#include <srrg_system_utils/chrono.h>
#include <stdio.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

#include "kdtree.cuh"
#include "surfel_matches.h"
#include "surfelv2.h"

namespace structure_refinement {
using ContainerType = std::vector<Eigen::Vector3d>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType *;

class DataAssociation {
 public:
  DataAssociation() {}
  ~DataAssociation() {}

  void associateDataKernelCPU(int, int, int, TreeNodeTypePtr *, TreeNodeTypePtr *, SurfelMatches *);
  void prepareDataCPU(std::vector<std::shared_ptr<TreeNodeType>> &kdTrees, std::vector<std::vector<TreeNodeTypePtr>> &kdTreeLeafes);
  void processTheSurfelMatches(std::vector<SurfelMatches> &);
  void resetSurfels();
  std::vector<Surfelv2> &getSurfels() { return surfels_; }

 private:
  std::vector<Surfelv2> surfels_;
};

}  // namespace structure_refinement
