#pragma once

#include <math.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

namespace structure_refinement {
using ContainerType = std::vector<Eigen::Vector3f>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType*;

struct SurfelMatches {
  TreeNodeTypePtr surfelA;
  TreeNodeTypePtr surfelB;
  bool matched;
};

}  // namespace structure_refinement
