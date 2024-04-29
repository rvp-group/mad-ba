#include "surfel.h"

namespace structure_refinement {
unsigned int Surfel::idCounter = 0;

Surfel::Surfel() {
  id_ = idCounter++;
}
Surfel::~Surfel() {}
bool Surfel::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
  return true;
}

void Surfel::addObservation(Eigen::Isometry3d& pose, unsigned int poseId, Eigen::Matrix<double, 3, 5>& observation) {
  // Sometimes the same pose and observation might be added twice
  // It hapens when two surfels from cloud i have the same corresponding surfel in cloud j
  odomPoses_.push_back(pose);
  observations_.push_back(observation);
  odomPosesIds_.push_back(poseId);
}

bool Surfel::checkIfsurfelIdExists(unsigned int poseId, unsigned int leafId) {
  // Check if that poseId exists in the map
  if (poseSurfelsIds_.count(poseId)) {
    // Check if leafId exists in the corresponding poseId (kdTree)
    if (poseSurfelsIds_.at(poseId).find(leafId) != poseSurfelsIds_.at(poseId).end())
      return true;
  }
  return false;
}

bool Surfel::checkIfPoseExists(unsigned int poseId) {
  // Check if that poseId exists in the map
  if (poseSurfelsIds_.count(poseId))
    return true;
  return false;
}

float Surfel::getLargestRadius() {
  float maxRadius = 0.1;
  for (Eigen::Matrix<double, 3, 5> observ : observations_) {
    float tmpRadius = std::sqrt(observ(1, 4) * observ(1, 4) + observ(2, 4) * observ(2, 4));
    if (tmpRadius > maxRadius)
      maxRadius = tmpRadius;
  }
  return maxRadius;
}

void SynthSurfel::addObservation(Eigen::Matrix<double, 3, 2>& observation) {
  // Sometimes the same pose and observation might be added twice
  // It hapens when two surfels from cloud i have the same corresponding surfel in cloud j
  observations_.push_back(observation);
}

Eigen::Isometry3f SynthSurfel::getIsometry() {
  Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
  iso.translation() = this->estPosition_.cast<float>();
  iso.linear() = matrixBetween2Vectors(Eigen::Vector3d(0, 0, 1), this->estNormal_).cast<float>();
  return iso;
}

Eigen::Matrix3d SynthSurfel::matrixBetween2Vectors(Eigen::Vector3d a, Eigen::Vector3d b) {
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
    std::cout << "doesn't work if a == -b" << std::endl;
  }
  return r;
}

}  // namespace structure_refinement