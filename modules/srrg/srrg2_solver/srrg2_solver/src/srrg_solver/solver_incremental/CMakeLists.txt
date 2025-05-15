add_library(srrg2_solver_incremental_library SHARED
  factor_graph_incremental_sorter.cpp
  factor_graph_incremental_sorter.h
  solver_incremental_base.cpp
  solver_incremental_base.h
  solver_incremental.cpp
  solver_incremental.h
  solver_incremental_running_environment.h
  solver_incremental_running_environment.cpp
  solver_incremental_runner.h
  solver_incremental_runner.cpp
  
  # instances
  instances.cpp instances.h
  )

target_link_libraries(srrg2_solver_incremental_library
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_core_library
  srrg2_solver_factor_graph_utils_library
  )
