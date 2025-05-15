#pragma once
#include "sim_sensor_3d_points_base.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  
  struct SimSensor3DPointsFOVLimited: public SimSensor3DPointsFOVLimitedBase {
    using FactorType=SE3PosePointOffsetErrorFactor;
    bool computeMeasurementInSensor(Eigen::Vector3f& measurement,
                                    const Eigen::Vector3f& world_point);

    void computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) override;
  };
  
}
