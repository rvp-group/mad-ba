#pragma once

#include <Eigen/Core>

namespace structure_refinement {

Eigen::Matrix3f matrixBetween2Vectors(Eigen::Vector3f a, Eigen::Vector3f b);
float angleBetween2Vectors(const Eigen::Vector3f& a, const Eigen::Vector3f& b);

}