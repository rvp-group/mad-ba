#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_property/property_eigen.h>
#include <srrg_property/property_vector.h>

#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_pcl/normal_computator.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/active_drawable.h>
#include <srrg_messages/messages/point_cloud2_message.h>

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

namespace structure_refinement {
  using namespace srrg2_core;

  class Surfel : public srrg2_core::MessageSinkBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Surfel();
    virtual ~Surfel();
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg) override;
    void addObservation(Eigen::Isometry3d &, unsigned int, Eigen::Matrix<double, 9, 1> &);  // Add new observation
    bool checkIfsurfelIdExists(unsigned int, unsigned int); // Checks whether such leafId was already associated

  //  protected:

    unsigned int id_;
    // double radius_;
    // Eigen::Vector3d normal_;
    // Eigen::VectorXd uncertainty_;
    std::map<unsigned int, std::set<unsigned int>> posesIds_;    // Corresponding id's of surfels in given poses | PoseId -> SurfelId its (id in kdTree vector)
    std::vector<std::shared_ptr<Eigen::Isometry3d>> poses_;      // Poses from which the surfel was observed
    std::vector<std::shared_ptr<Eigen::Matrix<double, 9, 1>>> observations_;  // Observations from each pose: mean (3DOF), normal (3DOF), bbox (1DOF)
    // std::vector<Eigen::Vector3d> points_;
  };

  using SurfelPtr = std::shared_ptr<Surfel>;

} // namespace structure_refinement
