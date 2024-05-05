add_executable(app_point_cloud_proc app_point_cloud_proc.cpp)
add_executable(main_app main_app.cpp)
add_executable(convert_gt_map convert_gt_map.cpp)


target_link_libraries(app_point_cloud_proc
    srrg2_solver_core_library
    srrg2_point_cloud_library
    srrg2_messages_library
    srrg2_data_structures_library
    srrg2_solver_types3d_library
    srrg2_image_library
    srrg2_config_library
    srrg2_property_library
    srrg2_boss_library
    srrg2_system_utils_library
    srrg2_viewer_library 
    srrg2_configurable_shell_library
    structure_refinement_library
    ${OpenCV_LIBS}
    ${QGLViewer_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${SRRG_QT_LIBRARIES}
    ${catkin_LIBRARIES}
    -lGL
)

target_link_libraries(main_app
    srrg2_solver_core_library
    srrg2_point_cloud_library
    srrg2_messages_library
    srrg2_data_structures_library
    srrg2_solver_types3d_library
    srrg2_image_library
    srrg2_config_library
    srrg2_property_library
    srrg2_boss_library
    srrg2_system_utils_library
    srrg2_viewer_library 
    srrg2_configurable_shell_library
    structure_refinement_library
    librviz_visual_tools.so
    ${OpenCV_LIBS}
    ${QGLViewer_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${SRRG_QT_LIBRARIES}
    ${catkin_LIBRARIES}
    -lGL
)

target_link_libraries(convert_gt_map
    srrg2_solver_core_library
    srrg2_point_cloud_library
    srrg2_messages_library
    srrg2_data_structures_library
    srrg2_solver_types3d_library
    srrg2_image_library
    srrg2_config_library
    srrg2_property_library
    srrg2_boss_library
    srrg2_system_utils_library
    srrg2_viewer_library 
    srrg2_configurable_shell_library
    structure_refinement_library
    librviz_visual_tools.so
    ${GLUT_LIBRARIES}
    ${SRRG_QT_LIBRARIES}
    ${catkin_LIBRARIES}
    -lGL
)