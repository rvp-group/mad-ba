#include "sim_sensor_pinhole_camera.h"
#include <cstdlib>
#include <iostream>
namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  
  /********************** SimSensorPinholeCamera **********************/

  SimSensorPinholeCamera::SimSensorPinholeCamera() {
    Eigen::Matrix3f cam_mat;
    cam_mat <<
      100, 0, 320,
      0, 100, 240,
      0, 0, 1;
    param_camera_matrix.setValue(cam_mat);
  }
  
  VariableBase::Id SimSensorPinholeCamera::setupConstants(FactorGraph& graph, VariableBase::Id start_idx) {
    start_idx=SimSensor3DPointsBase::setupConstants(graph, start_idx);
    Eigen::Matrix3f camera_matrix=param_camera_matrix.value();
    _inverse_camera_matrix=camera_matrix.inverse();
    projection_matrix_constant.reset(new VariableMatrix3_4);
    projection_matrix_constant->setGraphId(start_idx);
    auto est=projection_matrix_constant->estimate();
    auto inv_offset=sensor_offset_constant->estimate().inverse();
    est.block<3,3>(0,0)=camera_matrix*inv_offset.linear();
    est.block<3,1>(0,3)=camera_matrix*inv_offset.translation();
    projection_matrix_constant->setEstimate(est);
    projection_matrix_constant->setStatus(VariableBase::Fixed);
    graph.addVariable(projection_matrix_constant);
    ++start_idx;

    image_sizes_constant.reset(new VariablePoint2);
    image_sizes_constant->setGraphId(start_idx);
    image_sizes_constant->setEstimate(param_image_sizes.value());
    image_sizes_constant->setStatus(VariableBase::Fixed);
    graph.addVariable(image_sizes_constant);
    ++start_idx;
    return start_idx;
  }

  
  bool SimSensorPinholeCamera::spawnLandmark(Eigen::Vector3f& landmark) const {
    float range=drand48()*(param_max_range.value()-param_min_range.value())+param_min_range.value();
    Vector3f image_point(param_image_sizes.value().x()*drand48(),
                          param_image_sizes.value().y()*drand48(),
                          1);
    Vector3f local_point=_inverse_camera_matrix*image_point;
    local_point.normalize();
    local_point*=range;
    landmark = _sensor_pose*local_point;
    return true;
  }


  bool SimSensorPinholeCamera::computeMeasurementInSensor(Eigen::Vector2f& measurement,
                                                    const Eigen::Vector3f& point) const {

    Eigen::Vector3f local_point;
    if (! SimSensor3DPointsBase::computePointInSensor(local_point, point))
      return false;
    if (local_point.z()<param_min_range.value() || local_point.z()>param_max_range.value())
      return false;
    Eigen::Vector3f p_img=param_camera_matrix.value()*local_point;
    p_img.head<2>()*=(1./p_img.z());
    measurement=p_img.head<2>();
    if (measurement.x()<0
        || measurement.x()>param_image_sizes.value().x()
        || measurement.y()<0
        || measurement.y()>param_image_sizes.value().y())
      return false;
    return true;
  }

    void SimSensorPinholeCamera::computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) {
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
    //cerr << "size: " << matches.size() << " ";
    for (auto p_id: matches) {
      Vector3f world_point=world->landmarks[p_id];
      Vector2f measurement;
      if (! computeMeasurementInSensor(measurement, world_point)) {
        //cerr << "f";
        continue;
      }
      //cerr << endl;
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
      f->setVariableId(2, projection_matrix_constant->graphId());
      f->setVariableId(3, image_sizes_constant->graphId());
      
      f->setMeasurement(measurement);
      graph.addFactor(f);
    }
  }

}
