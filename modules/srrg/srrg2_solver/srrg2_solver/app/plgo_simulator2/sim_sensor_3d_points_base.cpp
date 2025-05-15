#include "sim_sensor_3d_points_base.h"
#include <cstdlib>

namespace srrg2_solver {
  using namespace srrg2_core;
  /********************** SimSensor3DPointsBase **********************/

  SimSensor3DPointsBase::SimSensor3DPointsBase() {
    Isometry3f landmark_sensor_offset;
    landmark_sensor_offset.setIdentity();
    landmark_sensor_offset.translation()=Vector3f(0,0,0.5);
    landmark_sensor_offset.linear() <<
      0, 0, 1,
      -1, 0, 0,
      0, -1, 0;
    param_sensor_offset.setValue(landmark_sensor_offset);
  }

  VariableBase::Id SimSensor3DPointsBase::setupConstants(FactorGraph& graph, VariableBase::Id start_idx) {
    sensor_offset_constant.reset(new VariableSE3QuaternionRight);
    sensor_offset_constant->setGraphId(start_idx);
    sensor_offset_constant->setEstimate(param_sensor_offset.value());
    sensor_offset_constant->setStatus(VariableBase::Fixed);
    graph.addVariable(sensor_offset_constant);
    ++ start_idx;
    return start_idx;
  }

  int SimSensor3DPointsBase::populateWorld() {
    for (int i=0; i<param_landmarks_per_frame.value(); ++i) {
      Eigen::Vector3f landmark;
      if (! spawnLandmark(landmark))
        return i;
      param_sensor_mode.value()->addLandmark(landmark);
    }
    return param_landmarks_per_frame.value();
  }

  bool SimSensor3DPointsBase::computePointInSensor(Eigen::Vector3f& local_point,
                                               const Eigen::Vector3f& point) const {
    local_point = _inv_sensor_pose*point;
    float range=local_point.norm();
    return range<param_max_range.value() && range>param_min_range.value();
  }

  /********************** SimSensor3DPointsFOVLimitedBase **********************/

  bool SimSensor3DPointsFOVLimitedBase::spawnLandmark(Eigen::Vector3f& landmark) const {
    float range=drand48()*(param_max_range.value()-param_min_range.value())+param_min_range.value();
    float az=(drand48()-.5)*param_horizontal_fov.value();
    float el=(drand48()-.5)*param_vertical_fov.value();
    Matrix3f Rz=geometry3d::rotationZ<float>(az);
    Matrix3f Ry=geometry3d::rotationY<float>(el);
    Vector3f p_sensor=Rz*Ry*Vector3f(0, 0, range);
    landmark = _sensor_pose*p_sensor;
    return true;
  }

  bool SimSensor3DPointsFOVLimitedBase::computePointInSensor(Eigen::Vector3f& local_point,
                                                             const Eigen::Vector3f& point) const {
    if (! SimSensor3DPointsBase::computePointInSensor(local_point, point))
      return false;
    float azimuth= atan2(local_point.y(), local_point.x());
    if (fabs(azimuth)>param_horizontal_fov.value())
      return false;
    float elevation=atan2(local_point.z(), local_point.head<2>().norm());
    if (fabs(elevation)>param_vertical_fov.value())
      return false;
    return true;
  }
}
