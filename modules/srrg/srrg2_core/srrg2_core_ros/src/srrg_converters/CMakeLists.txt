add_library(srrg2_converters_library SHARED
  converter.cpp
) 

target_link_libraries(srrg2_converters_library
  srrg2_messages_library
  srrg2_data_structures_library
  ${catkin_LIBRARIES}
)

add_dependencies(srrg2_converters_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
