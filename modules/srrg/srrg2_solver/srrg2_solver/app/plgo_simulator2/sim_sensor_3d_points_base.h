#pragma once
#include "sim_sensor_base.h"
#include "sim_sensor_mode_3d_landmarks.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  
  struct SimSensor3DPointsBase: public SimSensorBase {
    PARAM(PropertyEigen_<Eigen::Isometry3f>,
          sensor_offset,
          "position of the device w.r.t. the mobile platform",
          Eigen::Isometry3f::Identity(),
          0);

     PARAM(PropertyFloat,
          min_range,
          "min_range at which the points are seen",
          0.f,
          0);
    PARAM(PropertyFloat,
          max_range,
          "max_range at which the points are seen",
          10.f,
          0);
    PARAM(PropertyConfigurable_<SimSensorMode3DLandmarks>,
          sensor_mode,
          "slice of the world where to spawn the points",
          std::shared_ptr<SimSensorMode3DLandmarks>(new SimSensorMode3DLandmarks),
          nullptr);
    PARAM(PropertyInt,
          landmarks_per_frame,
          "number of landmarks potentially observed by this sensor for each frame",
          3,
          nullptr);
    SimSensor3DPointsBase();
    void setRobotPose(const Eigen::Isometry3f& robot_pose) override {
      SimSensorBase::setRobotPose(robot_pose);
      _sensor_pose=robot_pose*param_sensor_offset.value();
      _inv_sensor_pose=_sensor_pose.inverse();
    }

    VariableBase::Id setupConstants(FactorGraph& graph, VariableBase::Id start_idx) override;
    virtual bool spawnLandmark(Eigen::Vector3f& landmark) const = 0;
    virtual bool computePointInSensor(Eigen::Vector3f& local_point, const Eigen::Vector3f& point) const;
    int populateWorld() override;

    std::shared_ptr<VariableSE3QuaternionRight> sensor_offset_constant;
    Eigen::Isometry3f _sensor_pose=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f _inv_sensor_pose=Eigen::Isometry3f::Identity();


  };
  using SimSensor3DPointsBasePtr=std::shared_ptr<SimSensor3DPointsBase>;

  
  struct SimSensor3DPointsFOVLimitedBase: public SimSensor3DPointsBase {
    PARAM(PropertyFloat,
          horizontal_fov,
          "horizontal fov in radians",
          2.f,
          0);
    PARAM(PropertyFloat,
          vertical_fov,
          "vertical fov in radians",
          1.f,
          0);
    bool computePointInSensor(Eigen::Vector3f& local_point, const Eigen::Vector3f& point) const override;
    bool spawnLandmark(Eigen::Vector3f& landmark) const override;
  };
}
