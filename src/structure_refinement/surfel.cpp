#include "surfel.h"

namespace structure_refinement {

Surfel::Surfel() {
    
}
Surfel::~Surfel() {}
bool Surfel::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    return true;
}

void Surfel::addObservation(Eigen::Isometry3d& pose, unsigned int poseId, Eigen::Matrix<double, 9, 1>& observation) {
    // poseId is to check if the same correspondence is not added two times
    // if (posesId_.find(poseId) != posesId_.end())
    //     return;
    // else
    //     posesId_.insert(poseId);
}

bool Surfel::checkIfsurfelIdExists(unsigned int poseId, unsigned int leafId) {
    // Check if that poseId exists in the map
    if (posesIds_.count(poseId)) {
        // Check if leafId exists in the corresponding poseId (kdTree)
        if (posesIds_.at(poseId).find(leafId) != posesIds_.at(poseId).end())
            return true;
    }
    return false;
}

}  // namespace structure_refinement