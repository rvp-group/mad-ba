add_library(srrg2_solver_types_library SHARED
  variable_surfel.cpp
  variable_surfel.h
  variable_surfel_ad.cpp
  variable_surfel_ad.h
  se3_pose_surfel_factor_ad.cpp
  se3_pose_surfel_factor_ad.h
  instances.h
  instances.cpp
)


target_link_libraries(srrg2_solver_types_library
  srrg2_solver_types3d_library
  srrg2_solver_types_common_library
  srrg2_solver_core_library
)  
