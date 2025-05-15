#include "instances.h"
#include "trajectory_generator.h"
#include "sim_robot.h"
#include "sim_sensor_mode_3d_landmarks.h"
#include "sim_sensor_mode_3d_poses.h"
#include "sim_sensor_3d_points_fov_limited.h"
#include "sim_sensor_3d_poses.h"
#include "sim_sensor_omnicam.h"
#include "sim_sensor_pinhole_camera.h"
#include "sim_sensor_rectified_stereo.h"

namespace srrg2_solver {
  void srrg2_solver_plgo_simulator_register_types() {
    BOSS_REGISTER_CLASS(TrajectoryGeneratorSphere);
    BOSS_REGISTER_CLASS(TrajectoryGeneratorSphereUniform);
    BOSS_REGISTER_CLASS(TrajectoryGeneratorRandomManhattan);
    BOSS_REGISTER_CLASS(TrajectoryGeneratorTorus);
    BOSS_REGISTER_CLASS(SimRobot);
    BOSS_REGISTER_CLASS(SimSensorMode3DPoses);
    BOSS_REGISTER_CLASS(SimSensorMode3DLandmarks);
    BOSS_REGISTER_CLASS(SimSensor3DPoses);
    BOSS_REGISTER_CLASS(SimSensor3DPointsFOVLimited);
    BOSS_REGISTER_CLASS(SimSensorOmnicam);
    BOSS_REGISTER_CLASS(SimSensorPinholeCamera);
    BOSS_REGISTER_CLASS(SimSensorRectifiedStereo);
  }
};
