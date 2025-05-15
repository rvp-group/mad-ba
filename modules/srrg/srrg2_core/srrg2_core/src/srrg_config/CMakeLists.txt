 add_library(srrg2_config_library SHARED
   preemptible.cpp preemptible.h
   configurable.cpp  configurable.h
)
target_link_libraries(srrg2_config_library
  srrg2_property_library
  srrg2_boss_library
  srrg2_system_utils_library
  ${OpenCV_LIBS}
  )


add_library(srrg2_configurable_shell_library SHARED
  pipeline_runner.cpp
  pipeline_runner.h
  
  configurable_shell.cpp
  configurable_shell.h

  linenoise.c
  linenoise.h
  )

target_link_libraries(
  srrg2_configurable_shell_library
  srrg2_property_library
  srrg2_messages_library
  srrg2_viewer_library
  srrg2_config_library
  srrg2_boss_library
  )

add_executable(preemptible_example preemptible_example.cpp)

target_link_libraries(preemptible_example
  srrg2_configurable_shell_library
  ${catkin_LIBRARIES}
)
