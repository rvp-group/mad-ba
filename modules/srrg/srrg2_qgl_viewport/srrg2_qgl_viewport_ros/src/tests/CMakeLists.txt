add_executable(test_viewer_manager_server test_viewer_manager_server.cpp)
target_link_libraries(test_viewer_manager_server
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  srrg2_qgl_viewport_ros_library
  ${catkin_LIBRARIES}
  pthread
  )
  

add_executable(test_viewer_manager_client test_viewer_manager_client.cpp)
target_link_libraries(test_viewer_manager_client
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  srrg2_qgl_viewport_ros_library
  ${catkin_LIBRARIES}
  pthread
  )

  
add_executable(test_viewer_core_server test_viewer_core_server.cpp)
target_link_libraries(test_viewer_core_server
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  srrg2_qgl_viewport_ros_library
  ${catkin_LIBRARIES}
  pthread
  )
  
add_executable(test_viewer_core_server_multicanvas test_viewer_core_server_multicanvas.cpp)
target_link_libraries(test_viewer_core_server_multicanvas
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  srrg2_qgl_viewport_ros_library
  ${catkin_LIBRARIES}
  pthread
  )
  

add_executable(test_viewer_core_client test_viewer_core_client.cpp)
target_link_libraries(test_viewer_core_client
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  srrg2_qgl_viewport_ros_library
  ${catkin_LIBRARIES}
  pthread
  )
  