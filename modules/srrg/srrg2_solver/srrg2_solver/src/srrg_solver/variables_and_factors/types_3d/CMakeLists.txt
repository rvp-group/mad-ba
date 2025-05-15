add_library(srrg2_solver_types3d_library SHARED
  #ia variable stuff
  
  variable_matchable.cpp variable_matchable.h
  variable_se3.cpp       variable_se3.h
  variable_se3_ad.cpp    variable_se3_ad.h
  variable_point3.h 
  variable_point3_ad.h
  imu_bias_variable.cpp imu_bias_variable.h
  
  # tg correspondence driven factors
  se3_plane2plane_error_factor.cpp se3_plane2plane_error_factor.h
  se3_point2point_error_factor.cpp se3_point2point_error_factor.h 
  se3_matchable2matchable_error_factor.cpp se3_matchable2matchable_error_factor.h
  
  # tg pose-landmark factors 
  se3_pose_matchable_error_factor.cpp se3_pose_matchable_error_factor.h
  se3_pose_point_offset_error_factor.cpp se3_pose_point_offset_error_factor.h 
  se3_pose_point_error_factor.cpp se3_pose_point_error_factor.h 
  
  # tg prior factors
  se3_prior_error_factor_ad.cpp se3_prior_error_factor_ad.h
  se3_prior_offset_error_factor_ad.cpp se3_prior_offset_error_factor_ad.h
  # tg pose-pose factors
  se3_pose_pose_geodesic_error_factor.cpp se3_pose_pose_geodesic_error_factor.h
  se3_pose_pose_chordal_error_factor.cpp se3_pose_pose_chordal_error_factor.h
  se3_pose_pose_chordal_hessian_factor.cpp se3_pose_pose_chordal_hessian_factor.h
  # tg pose-pose factor
  se3_pose_pose_right_chordal_error_factor.cpp se3_pose_pose_right_chordal_error_factor.h

  # ldg imu ang gps factor stuff 
  preintegrated_imu_measurements.cpp preintegrated_imu_measurements.h 
  imu_factor_ad.cpp     imu_factor_ad.h
  gps_factor_ad.cpp     gps_factor_ad.h
 
  instances.h
  instances.cpp
)


target_link_libraries(srrg2_solver_types3d_library
  srrg2_solver_types_common_library
  srrg2_solver_core_library
)  
