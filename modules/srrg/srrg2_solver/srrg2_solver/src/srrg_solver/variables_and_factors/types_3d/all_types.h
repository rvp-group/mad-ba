#pragma once

#include "variable_matchable.h"
#include "variable_point3.h"
#include "variable_point3_ad.h"
#include "variable_se3.h"
#include "variable_se3_ad.h"

#include "se3_matchable2matchable_error_factor.h"
#include "se3_plane2plane_error_factor.h"    // Done
#include "se3_point2point_error_factor.h"    // Done
#include "se3_pose_motion_error_factor_ad.h" // Done (se3_prior_offset_multi_error_factor_ad.h)
#include "se3_prior_error_factor_ad.h"
#include "se3_prior_offset_error_factor_ad.h"

#include "se3_pose_matchable_error_factor.h"
#include "se3_pose_point_error_factor.h"
#include "se3_pose_point_offset_error_factor.h"
#include "se3_pose_pose_chordal_error_factor.h" // left
#include "se3_pose_pose_chordal_hessian_factor.h"
#include "se3_pose_pose_geodesic_error_factor.h"
#include "se3_pose_pose_right_chordal_error_factor.h" // right

#include "gps_factor_ad.h"
#include "imu_factor_ad.h"
