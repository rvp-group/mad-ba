#pragma once
#include "sim_sensor_mode_base.h"
#include <srrg_property/property_eigen.h>
#include <srrg_config/property_configurable.h>
#include "srrg_solver/solver_core/factor_graph.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  struct SimRobot;

  struct SimSensorBase: public Configurable {

    virtual void setRobotPose(const Eigen::Isometry3f& robot_pose) {
      _robot_pose=robot_pose;
      _inv_robot_pose=robot_pose.inverse();
    }
    virtual VariableBase::Id setupConstants(FactorGraph& graph, VariableBase::Id start_idx) {return start_idx;}
    virtual int populateWorld() {return 0;}
    virtual void computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id){}
  protected:
    Eigen::Isometry3f _robot_pose=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f _inv_robot_pose=Eigen::Isometry3f::Identity();
  };

  using SimSensorBasePtr=std::shared_ptr<SimSensorBase>;
}
