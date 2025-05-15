#include "sim_sensor_rectified_stereo.h"
#include <iostream>
#include <cstdlib>

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;


  /********************** SimSensorRectifiedStereo **********************/

  
  VariableBase::Id SimSensorRectifiedStereo::setupConstants(FactorGraph& graph, VariableBase::Id start_idx) {
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

    extended_projection_matrix.block<3,4>(0,0)=projection_matrix_constant->estimate();
    extended_projection_matrix.block<1,4>(3,0)=projection_matrix_constant->estimate().block<1,4>(0,0);
    extended_projection_matrix(3,3) -= param_baseline.value();

    image_sizes_stereo_constant.reset(new VariablePoint3);
    Eigen::Vector3f image_sizes_stereo(param_image_sizes.value().x(),
                                       param_image_sizes.value().y(),
                                       param_baseline.value());
    image_sizes_stereo_constant->setGraphId(start_idx);
    image_sizes_stereo_constant->setEstimate(image_sizes_stereo);
    image_sizes_stereo_constant->setStatus(VariableBase::Fixed);
    graph.addVariable(image_sizes_stereo_constant);
    ++start_idx;
    return start_idx;
  }

  bool SimSensorRectifiedStereo::spawnLandmark(Eigen::Vector3f& landmark) const {
        float range=drand48()*(param_max_range.value()-param_min_range.value())+param_min_range.value();
    Vector3f image_point((param_image_sizes.value().x()-param_baseline.value())*drand48(),
                          param_image_sizes.value().y()*drand48(),
                          1);
    Vector3f local_point=_inverse_camera_matrix*image_point;
    local_point.normalize();
    local_point*=range;
    landmark = _sensor_pose*local_point;
    return true;
  }

  bool SimSensorRectifiedStereo::computeMeasurementInSensor(Eigen::Vector3f& measurement, const Eigen::Vector3f& point) const  {
    Eigen::Vector3f point_in_robot=_inv_robot_pose*point;
    auto point_in_cam   = extended_projection_matrix.block<4,3>(0,0)*point_in_robot
      + extended_projection_matrix.block<4,1>(0,3);

    const float xl=point_in_cam(0);
    const float xr=point_in_cam(3);
    const float y=point_in_cam(1);
    const float z=point_in_cam(2);

    if (z<param_min_range.value() || z>param_max_range.value())
      return false;
    
    const float iz=1./z;
    measurement <<
      xl *iz,
      y *iz,
      (xl-xr) *iz;
    return measurement.x() > 0 && measurement.y() > 0
      && measurement.x()<param_image_sizes.value().x() && measurement.y()<param_image_sizes.value().y()     && (measurement.x()-measurement.z()) >=0;

  }

  void SimSensorRectifiedStereo::computeEpoch(FactorGraph& graph, VariableBase::Id current_pose_id) {
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
      Vector3f measurement;
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
      f->setVariableId(3, image_sizes_stereo_constant->graphId());
      
      f->setMeasurement(measurement);
      graph.addFactor(f);
    }
  }

}
