add_library(mad_ba_library SHARED
  point_cloud_proc.cpp
  point_cloud_proc_visual.cpp
  surfel.cpp
  instances.cpp
  data_association.cpp
  utils.cpp
)

target_link_libraries(mad_ba_library
  srrg2_solver_core_library
  srrg2_point_cloud_library
  srrg2_messages_library
  srrg2_data_structures_library
  srrg2_solver_types3d_library
  srrg2_solver_types_library
  srrg2_image_library
  srrg2_converters_library
  librviz_visual_tools.so
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

set_target_properties(mad_ba_library
 PROPERTIES CUDA_SEPARABLE_COMPILATION OFF
 )
