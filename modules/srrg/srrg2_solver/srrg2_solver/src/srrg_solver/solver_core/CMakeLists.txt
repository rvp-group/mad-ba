add_subdirectory(internals)
add_library(srrg2_solver_core_library SHARED
  #common
  solver_action_base.h

  solver_action_draw.cpp
  solver_action_draw.h

  solver_stats.h
  solver_stats.cpp

  termination_criteria.h
  termination_criteria.cpp

  iteration_algorithm_base.h
  iteration_algorithm_base.cpp

  iteration_algorithm_gn.h
  iteration_algorithm_gn.cpp

  iteration_algorithm_lm.h
  iteration_algorithm_lm.cpp

  iteration_algorithm_dl.h
  iteration_algorithm_dl.cpp

  iteration_algorithm_ddl.h
  iteration_algorithm_ddl.cpp
  
  iteration_algorithm_gn_ls.h
  iteration_algorithm_gn_ls.cpp
  
  iteration_algorithm_gn_pd.h
  iteration_algorithm_gn_pd.cpp

  robustifier.h
  robustifier.cpp

  robustifier_policy.h
  robustifier_policy.cpp
  
  variable.h
  ad_variable.h

  factor_base.h
  factor_base.cpp

  factor.h

  factor_graph.h
  factor_graph.cpp
  factor_graph_interface.cpp
  factor_graph_view.cpp
  
  variable_ptr_tuple.h
  ad_error_factor.h
  ad_constraint_factor.h
  solver.h
  solver.cpp
  solver_action_base.h
  solver_action_base.cpp

  
  instances.h
  instances.cpp
  )

target_link_libraries(srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  )

