add_library(structure_refinement_library SHARED
  point_cloud_proc.cpp
  surfel.cpp
  instances.cpp
  data_association.cu
)

target_link_libraries(structure_refinement_library
  srrg2_solver_core_library
  srrg2_point_cloud_library
  srrg2_messages_library
  srrg2_data_structures_library
  srrg2_solver_types3d_library
  srrg2_solver_krzystof_library
  srrg2_image_library
  srrg2_converters_library
  librviz_visual_tools.so
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)

set_target_properties(structure_refinement_library
 PROPERTIES CUDA_SEPARABLE_COMPILATION ON
 )
