add_executable(se3_epipolar_ba_euler_left_factor_test se3_epipolar_ba_factor_test.cpp)
target_compile_definitions(se3_epipolar_ba_euler_left_factor_test PUBLIC __SIDE__=0)
target_compile_definitions(se3_epipolar_ba_euler_left_factor_test PUBLIC __TYPE__=0)
target_link_libraries(se3_epipolar_ba_euler_left_factor_test
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(se3_epipolar_ba_euler_right_factor_test se3_epipolar_ba_factor_test.cpp)
target_compile_definitions(se3_epipolar_ba_euler_right_factor_test PUBLIC __SIDE__=1)
target_compile_definitions(se3_epipolar_ba_euler_right_factor_test PUBLIC __TYPE__=0)
target_link_libraries(se3_epipolar_ba_euler_right_factor_test
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})


add_executable(se3_epipolar_ba_quaternion_left_factor_test se3_epipolar_ba_factor_test.cpp)
target_compile_definitions(se3_epipolar_ba_quaternion_left_factor_test PUBLIC __SIDE__=0)
target_compile_definitions(se3_epipolar_ba_quaternion_left_factor_test PUBLIC __TYPE__=1)
target_link_libraries(se3_epipolar_ba_quaternion_left_factor_test
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(se3_epipolar_ba_quaternion_right_factor_test se3_epipolar_ba_factor_test.cpp)
target_compile_definitions(se3_epipolar_ba_quaternion_right_factor_test PUBLIC __SIDE__=1)
target_compile_definitions(se3_epipolar_ba_quaternion_right_factor_test PUBLIC __TYPE__=1)
target_link_libraries(se3_epipolar_ba_quaternion_right_factor_test
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(off_line_monocular_sfm_test off_line_monocular_sfm_test.cpp)
target_compile_definitions(off_line_monocular_sfm_test PUBLIC __SIDE__=0)
target_link_libraries(off_line_monocular_sfm_test
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(plgo2sfm plgo2sfm.cpp  sfm_common.cpp)
target_link_libraries(plgo2sfm
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(sfm2essential sfm2essential.cpp sfm_common.cpp)
target_link_libraries(sfm2essential
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(essential2orientations essential2orientations.cpp sfm_common.cpp)
target_link_libraries(essential2orientations
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})
  
add_executable(essential2orientations_constrained essential2orientations_constrained.cpp sfm_common.cpp)
target_link_libraries(essential2orientations_constrained
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(essential2translations essential2translations.cpp sfm_common.cpp)
target_link_libraries(essential2translations
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(sfm2ba sfm2ba.cpp sfm_common.cpp)
target_link_libraries(sfm2ba
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})

add_executable(sfm2eba sfm2eba.cpp sfm_common.cpp)
target_link_libraries(sfm2eba
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_sfm_library
  srrg2_solver_projective_library
  srrg2_solver_types3d_library
  srrg2_solver_core_library
  srrg2_geometry_library
  srrg2_solver_incremental_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  ${catkin_LIBRARIES})
