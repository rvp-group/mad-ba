#pragma once
#include "sim_sensor_3d_points_base.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_pinhole_ba_error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  
  struct SimSensorPinholeCamera: public SimSensor3DPointsBase {
    using FactorType = SE3PosePointPinholeBAErrorFactor; 
    PARAM(PropertyEigen_<Eigen::Matrix3f>,
          camera_matrix,
          "Camera Matrix (fx, 0, cx; 0, fy, cy; 0, 0, 1)",
          Eigen::Matrix3f::Identity(),
          0);
    PARAM(PropertyEigen_<Eigen::Vector2f>,
          image_sizes,
          "Image sizes (cols, rows)",
          Eigen::Vector2f(640,480),
          0);
    SimSensorPinholeCamera();
    VariableBase::Id setupConstants(FactorGraph& graph, VariableBase::Id start_idx) override;
    bool spawnLandmark(Eigen::Vector3f& landmark) const override;
    bool computeMeasurementInSensor(Eigen::Vector2f& measurement,
                                    const Eigen::Vector3f& world_point) const;
    void computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) override;
    
  protected:
    Eigen::Matrix3f _inverse_camera_matrix=Eigen::Matrix3f::Identity();
    std::shared_ptr<VariableMatrix3_4> projection_matrix_constant;
    std::shared_ptr<VariablePoint2> image_sizes_constant;
  };
}
