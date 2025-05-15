#pragma once
#include "sim_sensor_3d_points_fov_limited.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  
  struct SimSensorOmnicam: public SimSensor3DPointsFOVLimitedBase {
    using FactorType=SE3PosePointOmniBAErrorFactor;
    bool computeMeasurementInSensor(Eigen::Vector3f& measurement,
                                    const Eigen::Vector3f& world_point);
    void computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) override;
  };
}
