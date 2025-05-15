#ia internal stuff
add_subdirectory(viewer_core_ros)

#ia exposed viewer manager library
add_library(srrg2_viewer_ros_library SHARED
  viewer_context_ros.h
  viewer_context_ros.cpp
  viewer_manager_ros.h
  viewer_manager_ros.cpp
  )
target_link_libraries(srrg2_viewer_ros_library
  srrg2_viewer_core_ros_library)
