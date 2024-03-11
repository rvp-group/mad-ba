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

nlohmann::json Surfel::getJson() {
    // Create Json object
    nlohmann::json j;

    // Add surfel id
    j["id"] = id_;

    // Add all poses
    std::vector<std::vector<double>> posesVec;
    for (auto& pose : odomPoses_) {
        std::vector<double> poseVec(pose.matrix().data(), pose.matrix().data() + pose.matrix().rows() * pose.matrix().cols());
        posesVec.push_back(poseVec);
    }
    j["poses"] = posesVec;

    // Add all observations
    std::vector<std::vector<double>> observsVec;
    for (auto& observ : observations_) {
        std::vector<double> observVec(observ.data(), observ.data() + observ.rows() * observ.cols());
        observsVec.push_back(observVec);
    }
    j["observations"] = observsVec;

    return j;
}

}  // namespace structure_refinement