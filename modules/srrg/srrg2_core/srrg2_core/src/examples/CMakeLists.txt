add_executable(boss_property_example boss_property_example.cpp)
target_link_libraries(boss_property_example
 srrg2_system_utils_library
 srrg2_boss_library
 srrg2_property_library
 ${catkin_LIBRARIES})

add_executable(boss_shared_example boss_shared_example.cpp)
target_link_libraries(boss_shared_example
 srrg2_system_utils_library
 srrg2_boss_library
 ${catkin_LIBRARIES})

add_executable(boss_indexed_blob_example boss_indexed_blob_example.cpp)
target_link_libraries(boss_indexed_blob_example
 srrg2_system_utils_library
 srrg2_boss_library
 ${catkin_LIBRARIES})

add_executable(boss_example boss_example.cpp)
target_link_libraries(boss_example
 srrg2_system_utils_library
 srrg2_boss_library
 ${catkin_LIBRARIES})


add_executable(field_pack_example
  field_pack_example.cpp
)

add_executable(container_pack_example
  container_pack_example.cpp
)

add_executable(image_loading image_loading.cpp)
target_link_libraries(image_loading
  srrg2_system_utils_library
  srrg2_image_library
  ${catkin_LIBRARIES})

add_executable(kdtree_example kdtree_example.cpp)
target_link_libraries(kdtree_example
  srrg2_system_utils_library)

add_executable(matrix_example matrix_example.cpp)
target_link_libraries(matrix_example
  srrg2_system_utils_library)

add_executable(message_sorter_example message_sorter_example.cpp)
target_link_libraries(message_sorter_example
  srrg2_system_utils_library
  srrg2_messages_library
  srrg2_property_library
  ${catkin_LIBRARIES}
  )

add_executable(message_synchronizer_example message_synchronizer_example.cpp)
target_link_libraries(message_synchronizer_example
  srrg2_system_utils_library
  srrg2_messages_library
  srrg2_property_library
  ${catkin_LIBRARIES})


add_executable(messages_read_example messages_read_example.cpp)
target_link_libraries(messages_read_example
  srrg2_system_utils_library
  srrg2_messages_library
  srrg2_property_library
  ${catkin_LIBRARIES})

add_executable(messages_write_example messages_write_example.cpp)
target_link_libraries(messages_write_example
  srrg2_system_utils_library
  srrg2_messages_library
  srrg2_property_library
  ${catkin_LIBRARIES})


add_executable(point_cloud_example point_cloud_example.cpp)
target_link_libraries(point_cloud_example
  srrg2_system_utils_library
  srrg2_point_cloud_library
  ${catkin_LIBRARIES})

add_executable(shell_colors_example shell_colors_example.cpp)
target_link_libraries(shell_colors_example
  srrg2_system_utils_library)

add_executable(command_line_example command_line_example.cpp)
target_link_libraries(command_line_example
  srrg2_system_utils_library)


add_executable(timestamp_filter_example timestamp_filter_example.cpp)
target_link_libraries(timestamp_filter_example
srrg2_system_utils_library)
