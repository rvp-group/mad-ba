#include "surfel.h"
#include "json.hpp"
using json = nlohmann::json;

namespace structure_refinement {
unsigned int Surfel::idCounter = 0;

Surfel::Surfel() {
    id_ = idCounter++;
    data = 32;
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
    
    // Sometimes the same pose and observation might be added twice
    // It hapens when two surfels from cloud i have the same corresponding surfel in cloud j
    poses_.push_back(pose);
    observations_.push_back(observation);
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
// using json = nlohmann::json;

// void to_json(json& j, const Surfel& S) {
//     j = json{{"Name", S.id_},
//              {"Level", S.data}};
// }
// void from_json(const json& j, Surfel& S) {
//     j.at("Name").get_to(S.id_);
//     j.at("Level").get_to(S.data);
// }

    void to_json(json& j, const Surfel& p) {
        j = json{{"name", p.id_}, {"address", p.data}};
    }

    void from_json(const json& j, Surfel& p) {
        j.at("name").get_to(p.id_);
        j.at("address").get_to(p.data);
    }

}  // namespace structure_refinement