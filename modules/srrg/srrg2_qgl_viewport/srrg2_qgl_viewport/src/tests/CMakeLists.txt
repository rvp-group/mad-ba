#ia shared memory tests
add_executable(test_viewport test_viewport.cpp)
target_link_libraries(test_viewport
  srrg2_qgl_viewport_library
  ${catkin_LIBRARIES}
  pthread
)

add_executable(test_viewer_manager_shared test_viewer_manager_shared.cpp)
target_link_libraries(test_viewer_manager_shared
  srrg2_qgl_viewport_library
  ${catkin_LIBRARIES}
  pthread
)

# ds FIXME
#add_executable(test_opencv_shared test_opencv_shared.cpp)
#target_link_libraries(test_opencv_shared
#  srrg2_qgl_viewport_library
#  ${catkin_LIBRARIES}
#  pthread
#)

#ia multithings  
add_executable(test_multicanvas test_multicanvas.cpp)
target_link_libraries(test_multicanvas
  srrg2_qgl_viewport_library
  ${catkin_LIBRARIES}
  pthread
)

add_executable(test_viewer_core_multicanvas test_viewer_core_multicanvas.cpp)
target_link_libraries(test_viewer_core_multicanvas
  srrg2_qgl_viewport_library
  ${catkin_LIBRARIES}
  pthread
)


#ia opencv - opengl cancer integration
add_executable(test_cv_gl_interaction test_cv_gl_interaction.cpp)
target_link_libraries(test_cv_gl_interaction
  srrg2_qgl_viewport_library
  ${catkin_LIBRARIES}
  pthread
)