add_executable(srrg_viewer_ros_client srrg_viewer_ros_client.cpp)
target_link_libraries(srrg_viewer_ros_client
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  srrg2_qgl_viewport_ros_library
  ${catkin_LIBRARIES}
  pthread
  )
  