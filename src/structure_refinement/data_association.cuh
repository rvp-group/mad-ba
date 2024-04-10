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
class DataAssociation {
public:
  DataAssociation() {}
  ~DataAssociation() {}

  __host__ void prepareData(std::vector<std::shared_ptr<TreeNodeType>> );
  __host__ void prepareDataExample();

};
}  // namespace structure_refinement
