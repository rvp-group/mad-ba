#include "utils.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace structure_refinement {

Eigen::Matrix3f matrixBetween2Vectors(Eigen::Vector3f a, Eigen::Vector3f b) {
  a = a / a.norm();
  b = b / b.norm();
  Eigen::Vector3f v  = a.cross(b);
  float s = v.norm();
  float c = a.dot(b);
  Eigen::Matrix3f vx;
  vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  Eigen::Matrix3f r = Eigen::Matrix3f::Identity(3, 3);
  if (s != 0) {
    r = r + vx + vx * vx * ((1 - c) / std::pow(s, 2));
  } else {
    // std::cout << "doesn't work if a == -b" << std::endl;
  }
  return r;
}

float angleBetween2Vectors(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return atan2(a.cross(b).norm(), a.dot(b));
}

}