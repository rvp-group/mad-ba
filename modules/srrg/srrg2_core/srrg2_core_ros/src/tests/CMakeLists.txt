add_executable(bag_creator_test test_bag_creator.cpp)
target_link_libraries(bag_creator_test
  ${catkin_LIBRARIES}
  )

add_executable(conversion_in_place_test test_conversion_in_place.cpp)
target_link_libraries(conversion_in_place_test
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(tf_listener_playback_test test_tf_listener_playback.cpp)
target_link_libraries(tf_listener_playback_test
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(tf_listener_node_test test_tf_listener_node.cpp)
target_link_libraries(tf_listener_node_test
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(generic_listener_node_test test_generic_listener_node.cpp)
target_link_libraries(generic_listener_node_test
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(get_all_topics_test test_get_all_topics.cpp)
target_link_libraries(get_all_topics_test
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(message_source_ros_test test_message_source_ros.cpp)
target_link_libraries(message_source_ros_test
  srrg2_messages_ros_library
  srrg2_system_utils_library
  ${catkin_LIBRARIES}
  )

add_executable(tf_publisher_node_test test_tf_publisher_node.cpp)
target_link_libraries(tf_publisher_node_test
  srrg2_messages_ros_library
  ${catkin_LIBRARIES}
  )

#ia goddamn platform tests  
add_executable(tf_from_source_test test_tf_from_source.cpp)
target_link_libraries(tf_from_source_test
  srrg2_messages_ros_library
  srrg2_system_utils_library
  ${catkin_LIBRARIES})
  
add_executable(message_source_platform_test test_message_source_platform.cpp)
target_link_libraries(message_source_platform_test
  srrg2_messages_ros_library
  srrg2_system_utils_library
  ${catkin_LIBRARIES})
  
#ia viewer stuff
# catkin_add_gtest(test_rviz test_rviz.cpp)
# target_link_libraries(test_rviz
#   srrg2_messages_ros_library
#   ${catkin_LIBRARIES})
  
add_executable(buffer_sink_ros_test test_buffer_sink_ros.cpp)
target_link_libraries(buffer_sink_ros_test
  srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES})
  
add_executable(buffer_source_ros_test test_buffer_source_ros.cpp)
target_link_libraries(buffer_source_ros_test
  srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES})
  
add_executable(buffer_message_test test_buffer_message.cpp)
target_link_libraries(buffer_message_test
  srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES})
