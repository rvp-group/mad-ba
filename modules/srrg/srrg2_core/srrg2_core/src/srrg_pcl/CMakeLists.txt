add_library(srrg2_point_cloud_library
  lidar_3d_utils/spacing.cpp
  point_descriptor_field.cpp
  instances.cpp instances.h
)

target_link_libraries(srrg2_point_cloud_library
  srrg2_boss_library
  srrg2_property_library
  srrg2_config_library
  srrg2_system_utils_library
  opencv_core #ds can be removed when cv::Mat descriptors are removed
)
