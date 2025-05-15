add_library(srrg2_viewer_core_ros_library SHARED
  buffer_publisher.h
  buffer_publisher.cpp
  buffer_subscriber.h
  buffer_subscriber.cpp
  buffer_sink_ros.h
  buffer_sink_ros.cpp
  buffer_source_ros.h
  buffer_source_ros.cpp)
target_link_libraries(srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES}) ##ia TODO polish this
  
#ia this tells catkin to build first the messege, then this shit 
#add_dependencies(srrg2_viewer_core_ros_library ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(srrg2_viewer_core_ros_library ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

  