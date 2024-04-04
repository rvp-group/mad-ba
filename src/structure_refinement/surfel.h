#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_pcl/normal_computator.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_property/property_eigen.h>
#include <srrg_property/property_vector.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/active_drawable.h>

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "json.hpp"

namespace structure_refinement {
using namespace srrg2_core;

// This should be rather named CorrespondingSurfels or sth
class Surfel : public srrg2_core::MessageSinkBase {
    static unsigned int idCounter;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Surfel();
    virtual ~Surfel();
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg) override;
    void addObservation(Eigen::Isometry3d&, unsigned int, Eigen::Matrix<double, 3, 5>&);  // Add new observation
    bool checkIfsurfelIdExists(unsigned int, unsigned int);                               // Checks whether such leafId was already associated
    bool checkIfPoseExists(unsigned int );
    float getLargestRadius();

    nlohmann::json getJson();

    //  protected:

    unsigned int id_;
    // double radius_;
    // Eigen::Vector3d normal_;
    // Eigen::VectorXd uncertainty_;
    std::map<unsigned int, std::set<unsigned int>> poseSurfelsIds_;  // Corresponding id's of surfels in given poses | PoseId -> SurfelId its (id in kdTree vector)
                                                               // It is used for matching other leafs from other kdTrees
    std::vector<Eigen::Isometry3d> odomPoses_;                 // Odometry poses from which the surfel was observed | Indices correspond to observations_
    std::vector<uint> odomPosesIds_;                           // IDs of odometry poses corresponding to this->odomPoses_  | Used to detect if given surfel has two observations from one pose | Indices correspond to poses_ and observations_
    std::vector<Eigen::Matrix<double, 3, 5>> observations_;    // Observations from each pose: surfel eigen vectors (3 cols * 3 rows = 9 el), surfel mean (1 col = 3 el), surfel bbox (1 col = 3 el)
                                                               // It might be enough to store only normal (first column of eigen vectors)
                                                               // std::vector<Eigen::Vector3d> points_;
};
using SurfelPtr = std::shared_ptr<Surfel>;


class SynthSurfel{
    static unsigned int idCounter;

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SynthSurfel() {};
    ~SynthSurfel() {};
    void addObservation(Eigen::Matrix<double, 3, 2>&);
    Eigen::Matrix3d matrixBetween2Vectors(Eigen::Vector3d, Eigen::Vector3d );

    Eigen::Isometry3f getIsometry();

//    protected:
    unsigned int id_;
    // double radius_;
    Eigen::Vector3d estPosition_; // Initial estimate of position
    Eigen::Vector3d estNormal_; //Initial estimate of normal

    // Eigen::Vector3d normal_;
    // Eigen::VectorXd uncertainty_;
    // std::vector<Eigen::Isometry3d> odomPoses_;                      // Odometry poses from which the surfel was observed | Indices correspond to observations_
    std::vector<Eigen::Matrix<double, 3, 2>> observations_;         // Normal and mean

};
using SynthSurfelPtr = std::shared_ptr<SynthSurfel>;


}  // namespace structure_refinement
