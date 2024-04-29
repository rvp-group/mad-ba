#include "utils.h"

namespace structure_refinement {

Eigen::Matrix3d matrixBetween2Vectors(Eigen::Vector3d a, Eigen::Vector3d b) {
  a = a / a.norm();
  b = b / b.norm();
  Eigen::Vector3d v = a.cross(b);
  float s = v.norm();
  float c = a.dot(b);
  Eigen::Matrix3d vx;
  vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity(3, 3);
  if (s != 0) {
    r = r + vx + vx * vx * ((1 - c) / std::pow(s, 2));
  } else {
    // std::cout << "doesn't work if a == -b" << std::endl;
  }
  return r;
}

double angleBetween2Vectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  double angle = atan2(a.cross(b).norm(), a.dot(b));
  // if (angle > M_PI / 2.0)
  //     return M_PI - angle;
  // else
  return angle;
}

}