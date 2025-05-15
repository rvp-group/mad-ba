#include "sim_sensor_3d_poses.h"
#include <iostream>

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  
  bool SimSensor3DPoses::computeMeasurementInSensor(Eigen::Isometry3f& measurement, const Eigen::Isometry3f& other_pose, VariableBase::Id id_other, VariableBase::Id id_current) const {
    if (id_other>=id_current)
      return false;
    auto delta=id_current-id_other;
    // other pose too old
    if (delta>param_max_past_pose_delta.value())
      return false;
    if (delta<param_min_past_pose_delta.value())
      return false;
    auto local_pose=_inv_robot_pose*other_pose;
    auto n=local_pose.translation().norm();
    if (n<param_min_range.value()
        || n>param_max_range.value())
      return false;
    Eigen::AngleAxisf aa(local_pose.linear());
    if (fabs(aa.angle())>param_max_angle.value())
      return false;
    measurement=local_pose;
    return true;
  }

  void SimSensor3DPoses::computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) {
    VariableSE3QuaternionRight* current_pose_var=dynamic_cast<VariableSE3QuaternionRight*>(graph.variable(current_pose_id));
    if (! current_pose_var) {
      cerr << "ID: " <<current_pose_id << endl;
      throw std::runtime_error("no current pose for gen");
    }
    setRobotPose(current_pose_var->estimate());
    if (! path)
      throw std::runtime_error("no path set");
    IntVector matches;
    path->findMatches(matches, current_pose_var->estimate(), param_max_range.value());
    for (auto p_id: matches) {
      auto other_id= path->graphId(p_id);
      if (other_id>=current_pose_id)
        continue;
      VariableSE3QuaternionRight* other_pose_var=dynamic_cast<VariableSE3QuaternionRight*>(graph.variable(other_id));
      if (! other_pose_var) {
        cerr << "ID: " <<other_id << endl;
        throw std::runtime_error("no other for gen");
      }
      Isometry3f measurement;
      if (! computeMeasurementInSensor(measurement,
                                       other_pose_var->estimate(),
                                       other_pose_var->graphId(),
                                       current_pose_var->graphId()))
        continue;
      std::shared_ptr<FactorType> f(new FactorType);
      f->setVariableId(1, current_pose_var->graphId());
      f->setVariableId(0, other_pose_var->graphId());
      f->setMeasurement(measurement.inverse());
      graph.addFactor(f);
    }
  }
}
