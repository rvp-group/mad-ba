add_executable(solver_app_noise_adder
  solver_app_noise_adder.cpp)

target_link_libraries(solver_app_noise_adder
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_calib_library
  srrg2_solver_projective_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(solver_app_info_overwrite
  solver_app_info_overwrite.cpp)

target_link_libraries(solver_app_info_overwrite
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(solver_app_graph_initializer
  solver_app_graph_initializer.cpp)

target_link_libraries(solver_app_graph_initializer
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_incremental_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_executable(solver_app_graph_sorter
  solver_app_graph_sorter.cpp)

target_link_libraries(solver_app_graph_sorter
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

#ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_graph_incremental_reader
  solver_app_graph_incremental_reader.cpp)

target_link_libraries(solver_app_graph_incremental_reader
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

#ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_graph_evaluator
  solver_app_graph_evaluator.cpp)

target_link_libraries(solver_app_graph_evaluator
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

#ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_graph_gt_generator
  solver_app_graph_gt_generator.cpp)

target_link_libraries(solver_app_graph_gt_generator
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
