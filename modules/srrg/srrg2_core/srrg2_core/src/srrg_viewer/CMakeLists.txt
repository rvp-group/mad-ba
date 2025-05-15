add_subdirectory(viewer_core)

add_library(srrg2_viewer_library SHARED
  viewer_core_base.cpp viewer_core_base.h
  drawable_base.cpp drawable_base.h
  active_drawable.cpp active_drawable.h
  viewer_canvas.cpp viewer_canvas.h
  viewer_context_base.cpp viewer_context_base.h
  viewer_context_shared.cpp viewer_context_shared.h
  viewer_manager_base.cpp viewer_manager_base.h
  viewer_manager_shared.cpp viewer_manager_shared.h
)

target_link_libraries(srrg2_viewer_library
  srrg2_viewer_core_library
  srrg2_system_utils_library
)
