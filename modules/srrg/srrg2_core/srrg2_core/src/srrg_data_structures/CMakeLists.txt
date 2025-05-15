add_library(srrg2_data_structures_library SHARED
  graph.cpp graph.h
  path_matrix.cpp path_matrix.h
  path_matrix_distance_search.cpp
  path_matrix_dijkstra_search.cpp
  path_matrix_clusterer_search.cpp
  path_matrix_astar_search.cpp
  path_matrix_cost_search.cpp
  link.cpp link.h
  platform.cpp platform.h
  events.h
  grid_map_2d.cpp grid_map_2d.h
  file_handle.cpp
)

target_link_libraries(srrg2_data_structures_library
  srrg2_image_library
  srrg2_property_library
  srrg2_system_utils_library
  srrg2_config_library
  )

add_executable(file_vector_interface_test file_vector_interface_test)

target_link_libraries( file_vector_interface_test
  srrg2_data_structures_library
  srrg2_image_library
  srrg2_property_library
  srrg2_system_utils_library
  srrg2_config_library
)
