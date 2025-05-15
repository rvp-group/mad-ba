#pragma once
#include "sim_sensor_base.h"
#include "sim_sensor_mode_3d_poses.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"


namespace srrg2_solver {
  using namespace srrg2_core;

  struct SimSensor3DPoses: public SimSensorBase{
    using FactorType = SE3PosePoseGeodesicErrorFactor; 
     PARAM(PropertyFloat,
          min_range,
          "min_range at which the poses are seen",
          0.f,
          0);
    PARAM(PropertyFloat,
          max_range,
          "max_range at which the other_poses are seen",
          10.f,
          0);
    PARAM(PropertyFloat,
          max_angle,
          "max angle between orientations to see a pose",
          10.f,
          0);
    PARAM(PropertyInt,
          min_past_pose_delta,
          "poses younger than current_pose - min_past_pose will be ignored",
          20,
          0);
    PARAM(PropertyInt,
          max_past_pose_delta,
          "poses older than current_pose - max_past_pose will be ignored",
          1000000,
          0);

    bool computeMeasurementInSensor(Eigen::Isometry3f& measurement,
                                    const Eigen::Isometry3f& other_pose,
                                    VariableBase::Id id_other,
                                    VariableBase::Id id_current) const;

    void computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) override;

    SimSensorMode3DPosesPtr path;

  };

  using SimSensor3DPosesPtr = std::shared_ptr<SimSensor3DPoses>;
    
}
