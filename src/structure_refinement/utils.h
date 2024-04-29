#pragma once

#include <iterator>
#include <Eigen/Core>

namespace structure_refinement {

Eigen::Matrix3d matrixBetween2Vectors(Eigen::Vector3d a, Eigen::Vector3d b);
double angleBetween2Vectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

}