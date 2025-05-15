add_executable(rosbag_to_srrg_converter_app rosbag_to_srrg_converter_app.cpp)
target_link_libraries(rosbag_to_srrg_converter_app
  srrg2_messages_ros_library
  ${catkin_LIBRARIES})

add_executable(srrg_to_rosbag_converter_app srrg_to_rosbag_converter_app.cpp)
target_link_libraries(srrg_to_rosbag_converter_app
  srrg2_system_utils_library
  srrg2_messages_ros_library
  ${catkin_LIBRARIES}
  )

add_executable(extract_gt_from_ros extract_gt_from_ros.cpp)
target_link_libraries(extract_gt_from_ros
  srrg2_messages_ros_library
  ${catkin_LIBRARIES})

  