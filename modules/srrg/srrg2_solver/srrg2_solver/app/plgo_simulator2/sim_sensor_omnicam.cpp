#include "sim_sensor_omnicam.h"
#include <iostream>

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  
  /********************** SimSensorOmnicam **********************/
  bool SimSensorOmnicam::computeMeasurementInSensor(Eigen::Vector3f& measurement,
                                                      const Eigen::Vector3f& world_point) {
    if (! SimSensor3DPointsFOVLimitedBase::computePointInSensor(measurement, world_point))
      return false;
    measurement.normalize();
    return true;
  }

  void SimSensorOmnicam::computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) {
    VariableSE3QuaternionRight* current_pose_var=dynamic_cast<VariableSE3QuaternionRight*>(graph.variable(current_pose_id));
    if (! current_pose_var) {
      cerr << "ID: " <<current_pose_id << endl;
      throw std::runtime_error("no current pose for gen");
    }
    setRobotPose(current_pose_var->estimate());
    SimSensorMode3DLandmarksPtr world=param_sensor_mode.value();
    if (! world) {
      throw std::runtime_error("no world for this sensor");
    }
    IntVector matches;
    world->findMatches(matches, _sensor_pose.translation(), param_max_range.value());
    for (auto p_id: matches) {
      Vector3f world_point=world->landmarks[p_id];
      Vector3f measurement;
      if (! computeMeasurementInSensor(measurement, world_point))
        continue;
      
      auto other_id= world->graphId(p_id);
      VariableBase* other_var=graph.variable(other_id);
      VariablePoint3* landmark = nullptr;
      if (! other_var) {
        landmark= new VariablePoint3;
        landmark->setEstimate(world_point);
        landmark->setGraphId(other_id);
        graph.addVariable(std::shared_ptr<VariablePoint3>(landmark));
        world->landmarks_num_times_seen[p_id]++;
      } else {
        landmark=dynamic_cast<VariablePoint3*>(other_var);
        if (! landmark)
          throw std::runtime_error("type mismatch");
      }
      std::shared_ptr<FactorType> f(new FactorType);
      f->setVariableId(0, current_pose_var->graphId());
      f->setVariableId(1, landmark->graphId());
      f->setVariableId(2, sensor_offset_constant->graphId());
      f->setMeasurement(measurement);
      graph.addFactor(f);
    }
  }


}
