#include "sim_robot.h"
#include <srrg_config/configurable_command.h>
#include <iostream>

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  
  SimRobot::SimRobot() {
    graph.reset(new FactorGraph);
    addCommand(
      new ConfigurableCommand_<SimRobot, typeof(&SimRobot::cmdSaveGraph), std::string, std::string>(
        this, "save", "saves a graph to a json file", &SimRobot::cmdSaveGraph));

    addCommand(
      new ConfigurableCommand_<SimRobot, typeof(&SimRobot::cmdSetup), std::string>(
        this, "setup", "prepares the simulation", &SimRobot::cmdSetup));

    addCommand(
      new ConfigurableCommand_<SimRobot, typeof(&SimRobot::cmdCompute), std::string>(
        this, "compute", "clears the initialization", &SimRobot::cmdCompute));

  }

  bool SimRobot::cmdSaveGraph(std::string& response, const std::string& filename) {
    response = "saving graph to file [" + filename + "]";
    saveGraph(filename);
    return true;
  }

  bool SimRobot::cmdSetup(std::string& response) {
    reset();
    setup();
    response = "setting up stuff";
    return true;
  }

  bool SimRobot::cmdCompute(std::string& response) {
    response = "computing";
    compute();
    return true;
  }

  void SimRobot::saveGraph(const std::string& filename) {
    graph->write(filename);
  }
  
  void SimRobot::reset() {
    param_trajectory_generator->reset();
    graph->clear();
    Configurable::reset();
    Preemptible::reset();
  }
  
  void SimRobot::setup() {
    
    //1. calls the trajectory generator 
    cerr << "generating trajectory...";
    param_trajectory_generator->compute();
    cerr << path().size() << "poses" << endl;

    //2. adds the constants for each sensor
    cerr << "adding constants... ";
    VariableBase::Id variable_id=path().size();
    for (size_t s=0; s<param_sensors.size(); ++s) {
      SimSensorBasePtr sensor=param_sensors.value(s);
      SimSensor3DPosesPtr pose_sensor=std::dynamic_pointer_cast<SimSensor3DPoses>(sensor);
      if (pose_sensor) {
        pose_sensor->path=param_path_sensor_mode.value();
      }
      variable_id=sensor->setupConstants(*graph, variable_id);
    }
    cerr << variable_id-path().size() << " var added" << endl;       

    cerr << "populating path slice .... ";
    //3. if PGO closures are needed populate the world view of poses
    if (param_path_sensor_mode.value()) {
      for (auto& p: path()) {
        param_path_sensor_mode.value()->addPose(p);
        //cerr << "adding pose " << p.translation().transpose() << endl;
      }
      param_path_sensor_mode.value()->setup();
    }
    cerr << " DONE" << endl;
    

    cerr << "Generating Landmarks.... ";
    //4. invokes the landmark generators for each sensor in the trajectory
    for (auto& p: path()) {
      for (size_t s=0; s<param_sensors.size(); ++s) {
        SimSensorBasePtr sensor=param_sensors.value(s);
        sensor->setRobotPose(p);
        sensor->populateWorld();
      }
    }
    cerr << " DONE" << endl;

    cerr << "Pruning Landmarks.... ";
    //5. for each landmark sensor, sparsifies the world slice
    std::set<SimSensorMode3DLandmarksPtr> sensor_modes_3d_landmarks;
    for (size_t s=0; s<param_sensors.size(); ++s) {
      SimSensor3DPointsBasePtr sensor=std::dynamic_pointer_cast<SimSensor3DPointsBase>(param_sensors.value(s));
      if (sensor) {
        sensor_modes_3d_landmarks.insert(sensor->param_sensor_mode.value());
      }
    } 
    cerr << "Pruning2.... ";
    for (auto m: sensor_modes_3d_landmarks) {
      m->setup();
      m->offset=variable_id;
      variable_id+=m->size();
    }
    
    

  }

  void SimRobot::compute() {
    reset();
    setup();
    VariableBase::Id pose_id=0;
    for (auto& p: path()) {
      std::shared_ptr<VariableSE3QuaternionRightAD> robot_pose(new VariableSE3QuaternionRightAD);
      robot_pose->setGraphId(pose_id);
      robot_pose->setEstimate(p);
      if (pose_id==0)
        robot_pose->setStatus(VariableBase::Fixed);
      graph->addVariable(robot_pose);
      for (size_t s=0; s<param_sensors.size(); ++s) {
        SimSensorBasePtr sensor=param_sensors.value(s);
        sensor->computeEpoch(*graph, pose_id);
        preemptGlobal();
      }
      ++pose_id;
    }

  }

}
