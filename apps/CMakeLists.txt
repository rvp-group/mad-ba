add_executable(main_app main_app.cpp)

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
    mad_ba_library
    librviz_visual_tools.so
    ${OpenCV_LIBS}
    ${QGLViewer_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${SRRG_QT_LIBRARIES}
    ${catkin_LIBRARIES}
    -lGL
)