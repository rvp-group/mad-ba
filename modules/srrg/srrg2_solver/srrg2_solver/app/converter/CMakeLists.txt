#ia converter app. given a g2o file it becames a boss graph and viceversa
add_library (srrg2_g2o_converter_library
  g2o_converter_action_base.h
  g2o_converter_action_se2.h
  g2o_converter_action_se2.cpp
  g2o_converter_action_se3.h
  g2o_converter_action_se3.cpp
  g2o_converter_actions.h
  g2o_converter.h
  g2o_converter.cpp
)

target_link_libraries(srrg2_g2o_converter_library
  srrg2_solver_types2d_library 
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_core_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(solver_app_graph_converter
  solver_app_graph_converter.cpp
)
target_link_libraries(solver_app_graph_converter
  srrg2_g2o_converter_library
  srrg2_solver_types2d_library 
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_core_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)




