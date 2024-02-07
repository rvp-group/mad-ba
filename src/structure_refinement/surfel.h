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

  protected:
    uint64_t id_pose_;
    uint64_t id_surfel_;
    float radius_;
    std::vector<Eigen::Vector3f> points_;
    Eigen::Vector3f normal_;
    Eigen::VectorXf uncertainty_;
  };

  using SurfelPtr = std::shared_ptr<Surfel>;

} // namespace structure_refinement
