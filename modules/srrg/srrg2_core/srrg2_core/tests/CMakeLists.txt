catkin_add_gtest(test_abstract_map test_abstract_map.cpp)

catkin_add_gtest(test_geometry test_geometry.cpp)

catkin_add_gtest(test_matrix_bilinear_interpolation test_matrix_bilinear_interpolation.cpp)

#ia TODO
catkin_add_gtest(test_image_data test_image_data.cpp)
target_link_libraries(test_image_data
  srrg2_system_utils_library
  srrg2_image_library
)

catkin_add_gtest(test_points test_points.cpp)
target_link_libraries(test_points
  srrg2_point_cloud_library
)

catkin_add_gtest(test_streaming_configurable test_streaming_configurable.cpp)
target_link_libraries(test_streaming_configurable
  srrg2_point_cloud_library
)

# catkin_add_gtest(test_message_sources test_message_sources.cpp)
# target_link_libraries(test_message_sources
#   srrg2_messages_library
# )

catkin_add_gtest(test_messages test_messages.cpp)
target_link_libraries(test_messages
  srrg2_messages_library
)

catkin_add_gtest(test_kd_tree test_kd_tree.cpp)
target_link_libraries(test_kd_tree
  srrg2_data_structures_library
)

catkin_add_gtest(test_point_intensity_descriptor test_point_intensity_descriptor.cpp)
target_link_libraries(test_point_intensity_descriptor
  srrg2_point_cloud_library
)

catkin_add_gtest(test_unprojector test_unprojector.cpp)
target_link_libraries(test_unprojector
  srrg2_point_cloud_library
)

catkin_add_gtest(test_projector test_projector.cpp)
target_link_libraries(test_projector
  srrg2_system_utils_library
  srrg2_point_cloud_library
)

catkin_add_gtest(test_point_statistics_field test_point_statistics_field.cpp)
target_link_libraries(test_point_statistics_field
  srrg2_point_cloud_library
)

catkin_add_gtest(test_viewer_packets test_viewer_packets.cpp)
target_link_libraries(test_viewer_packets
  srrg2_property_library
  srrg2_config_library
  srrg2_viewer_core_library
)

catkin_add_gtest(test_profiler test_profiler.cpp)
target_link_libraries(test_profiler
  srrg2_system_utils_library
)

#ia GTEST: matchable basics
catkin_add_gtest(test_matchables test_matchables.cpp)
target_link_libraries(test_matchables
  srrg2_system_utils_library
  srrg2_matchable_library)


catkin_add_gtest(test_similiarity test_similiarity.cpp)
target_link_libraries(test_similiarity)

catkin_add_gtest(test_epipolar test_epipolar.cpp)
target_link_libraries(test_epipolar
  srrg2_geometry_library)
