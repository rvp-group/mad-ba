add_library(srrg2_qgl_viewport_library SHARED
  gl_helpers.cpp
  qgl_packet_processor.cpp
  qgl_viewport.cpp
  viewer_core_base_qgl.cpp
  viewer_core_shared_qgl.cpp
)

target_link_libraries(srrg2_qgl_viewport_library
  srrg2_viewer_library
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
