add_executable(solver_app_plgo_simulator
  solver_app_plgo_simulator.cpp
  graph_generator.cpp
  plgo_simulator.cpp
  trajectory_generator.cpp
  sim_record.cpp)

target_link_libraries(solver_app_plgo_simulator
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  srrg2_solver_projective_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

