add_library(srrg2_qgl_viewport_ros_library SHARED
  viewer_core_ros_qgl.cpp viewer_core_ros_qgl.h
  )
target_link_libraries(srrg2_qgl_viewport_ros_library
  srrg2_viewer_ros_library
  srrg2_qgl_viewport_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES})
  
