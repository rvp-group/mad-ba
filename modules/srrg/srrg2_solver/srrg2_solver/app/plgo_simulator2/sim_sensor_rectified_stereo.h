#pragma once
#include "sim_sensor_pinhole_camera.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_rectified_stereo_error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  

  struct SimSensorRectifiedStereo: public SimSensorPinholeCamera{
    using FactorType = SE3PosePointRectifiedStereoErrorFactor; 
    PARAM(PropertyFloat, baseline, "baseline left to right, in pixels", 100, 0);
    VariableBase::Id setupConstants(FactorGraph& graph, VariableBase::Id start_idx) override;
    bool spawnLandmark(Eigen::Vector3f& landmark) const override;
    bool computeMeasurementInSensor(Eigen::Vector3f& measurement,
                                    const Eigen::Vector3f& world_point) const;
    void computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id);
    std::shared_ptr<VariablePoint3> image_sizes_stereo_constant;
    Eigen::Matrix<float,4,4> extended_projection_matrix;
  };

    
}
