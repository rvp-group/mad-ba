add_library(srrg2_solver_types_common_library SHARED
  instances.h
  instances.cpp
  )

target_link_libraries(srrg2_solver_types_common_library
  srrg2_solver_core_library
  )

