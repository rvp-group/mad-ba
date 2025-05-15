add_library(srrg2_solver_plgo_simulator_library
  trajectory_generator.cpp
  sim_sensor_mode_base.cpp
  sim_sensor_mode_3d_poses.cpp
  sim_sensor_mode_3d_landmarks.cpp
  sim_sensor_3d_points_base.cpp
  sim_sensor_3d_poses.cpp
  sim_sensor_3d_points_fov_limited.cpp
  sim_sensor_omnicam.cpp
  sim_sensor_pinhole_camera.cpp
  sim_sensor_rectified_stereo.cpp
  sim_robot.cpp
  instances.cpp)

target_link_libraries(srrg2_solver_plgo_simulator_library
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  srrg2_solver_projective_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)


# add_executable(solver_app_plgo_simulator
#   solver_app_plgo_simulator.cpp
#   graph_generator.cpp
#   plgo_simulator.cpp
#   trajectory_generator.cpp
#   sim_record.cpp)

# target_link_libraries(solver_app_plgo_simulator
#   srrg2_solver_types2d_library
#   srrg2_solver_types3d_library
#   srrg2_solver_calib_library
#   srrg2_solver_projective_library
#   ${SOLVER_EXAMPLES_CORE_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

