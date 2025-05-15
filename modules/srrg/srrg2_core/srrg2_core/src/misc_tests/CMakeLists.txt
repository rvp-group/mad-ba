add_executable(app_test_distance_map test_distance_map.cpp)
target_link_libraries(app_test_distance_map
  srrg2_property_library
  srrg2_config_library
  srrg2_image_library
  srrg2_point_cloud_library
  srrg2_data_structures_library
  srrg2_system_utils_library)

#add_executable(test_platform_synthetic_joints test_platform_synthetic_joints.cpp)
#target_link_libraries(test_platform_synthetic_joints
#  srrg2_system_utils_library
#  srrg2_data_structures_library
#)

#add_executable(test_platform_synthetic_transforms test_platform_synthetic_transforms.cpp)
#target_link_libraries(test_platform_synthetic_transforms
#  srrg2_system_utils_library
#  srrg2_data_structures_library
#)


#ia normals
add_executable(app_test_normal_computation test_normal_computation.cpp)
target_link_libraries(app_test_normal_computation
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library
  srrg2_data_structures_library
  srrg2_image_library
)

# #ia giorgio point test
add_executable(app_test_point_ops test_point_ops.cpp)
target_link_libraries(app_test_point_ops
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library
  srrg2_point_cloud_library
  )
  
add_executable(app_test_point_cloud_pinhole test_point_cloud_pinhole.cpp)
target_link_libraries(app_test_point_cloud_pinhole
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library
  srrg2_point_cloud_library
)

add_executable(app_test_point_cloud_polar test_point_cloud_polar.cpp)
target_link_libraries(app_test_point_cloud_polar
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library
  srrg2_point_cloud_library
)
  
  
#ia point processor - aka integral image
add_executable(app_test_point_matrix_processor test_point_matrix_processor.cpp)
target_link_libraries(app_test_point_matrix_processor
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library)

#bdc test intensity point
add_executable(app_test_cloud_intensity test_cloud_intensity.cpp)
target_link_libraries(app_test_cloud_intensity
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library
  srrg2_data_structures_library
  srrg2_image_library
)


#add_executable(test_key_events test_key_events.cpp)
#target_link_libraries(test_key_events
#  srrg2_system_utils_library
#  srrg2_data_structures_library
#  pthread
#)

# viewer things
add_executable(app_test_viewer_packets_serialization test_viewer_packets_serialization.cpp)
target_link_libraries(app_test_viewer_packets_serialization
  srrg2_property_library
  srrg2_config_library
  srrg2_viewer_core_library
)

add_executable(app_test_viewer_packets_factory test_viewer_packets_factory.cpp)
target_link_libraries(app_test_viewer_packets_factory
  srrg2_property_library
  srrg2_config_library
  srrg2_viewer_core_library
)

add_executable(app_test_viewer_buffer_manager test_viewer_buffer_manager.cpp)
target_link_libraries(app_test_viewer_buffer_manager
  srrg2_property_library
  srrg2_config_library
  srrg2_viewer_core_library
)

add_executable(app_test_viewer_shared_memory test_viewer_shared_memory.cpp)
target_link_libraries(app_test_viewer_shared_memory
  srrg2_property_library
  srrg2_config_library
  srrg2_viewer_core_library
  srrg2_system_utils_library
)


