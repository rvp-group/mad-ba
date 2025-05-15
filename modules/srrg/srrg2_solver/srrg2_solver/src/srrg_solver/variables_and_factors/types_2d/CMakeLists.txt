add_library(srrg2_solver_types2d_library SHARED
#   #variable types
  variable_se2.h variable_se2.cpp
  variable_so2.h variable_so2.cpp
  variable_se2_ad.h variable_se2_ad.cpp
  variable_so2_ad.h variable_so2_ad.cpp
  variable_point2.h
  variable_point2_ad.h

  se2_circumference_constraint_factor.cpp se2_circumference_constraint_factor.h
  se2_prior_error_factor.cpp se2_prior_error_factor.h
  se2_point2point_error_factor.cpp se2_point2point_error_factor.h
  se2_plane2plane_error_factor.cpp se2_plane2plane_error_factor.h
  
  se2_pose_pose_geodesic_error_factor.cpp se2_pose_pose_geodesic_error_factor.h
  se2_pose_pose_chordal_error_factor.cpp se2_pose_pose_chordal_error_factor.h
  se2_pose_point_error_factor.cpp se2_pose_point_error_factor.h
  se2_pose_point_bearing_error_factor.cpp se2_pose_point_bearing_error_factor.h
  se2_pose_point_range_constraint_factor.cpp se2_pose_point_range_constraint_factor.h
  
  instances.h
  instances.cpp
  )

target_link_libraries(srrg2_solver_types2d_library
  srrg2_solver_core_library
  srrg2_solver_types_common_library
  )

