# #ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_graph_optimizer solver_app_graph_optimizer.cpp)
target_link_libraries(solver_app_graph_optimizer
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)

# #ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_incremental_optimizer solver_app_incremental_optimizer.cpp)
target_link_libraries(solver_app_incremental_optimizer
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_incremental_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
)




