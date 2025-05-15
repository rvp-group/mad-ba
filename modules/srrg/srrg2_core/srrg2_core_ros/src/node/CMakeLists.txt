add_executable(srrg_to_ros_publisher_node srrg_to_ros_publisher_node.cpp)
target_link_libraries(srrg_to_ros_publisher_node
  srrg2_system_utils_library
  srrg2_messages_ros_library
  ${catkin_LIBRARIES}
  )

add_executable(ros_to_srrg_listener_node ros_to_srrg_listener_node.cpp)
target_link_libraries(ros_to_srrg_listener_node
  srrg2_system_utils_library
  srrg2_messages_ros_library
  ${catkin_LIBRARIES}
  )
