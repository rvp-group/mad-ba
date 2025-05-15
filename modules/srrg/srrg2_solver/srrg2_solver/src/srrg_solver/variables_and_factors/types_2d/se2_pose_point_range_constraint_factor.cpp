#include "srrg_solver/solver_core/constraint_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"
#include <srrg_solver/variables_and_factors/types_2d/se2_pose_point_range_constraint_factor.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2PosePointRangeConstraintFactor::constraintAndJacobian(bool error_only_) {
    const VariableSE2Right* pose = _variables.at<0>();
    const VariablePoint2* point  = _variables.at<1>();

    const Isometry2f& transformation = pose->estimate();
    const Vector2f& landmark         = point->estimate();
    const Matrix2f& R                = transformation.linear();

    const Vector2f predicted_point = transformation.inverse() * landmark;
    _constraint(0, 0)              = -predicted_point.norm() + _min_range;
    if (error_only_) {
      return;
    }

    // tg Jacobian of the bearing prediction with respect to the predicted point
    Vector2f J_norm = predicted_point;
    J_norm /= predicted_point.norm();
    auto J_pose              = jacobian<0>();
    J_pose.block<1, 2>(0, 0) = -J_norm.transpose();
    J_pose(0, 2)             = J_norm.transpose() * geometry2d::skew(predicted_point);
    J_pose                   = -J_pose;
    auto J_point             = jacobian<1>();
    J_point                  = J_norm.transpose() * R.transpose();
    J_point                  = -J_point;
  }

  INSTANTIATE(SE2PosePointRangeConstraintFactor)

} // namespace srrg2_solver
