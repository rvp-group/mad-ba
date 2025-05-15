#include "se2_circumference_constraint_factor.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/constraint_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE2CircumferenceConstraintFactor::constraintAndJacobian(bool constraint_only_) {
    const EstimateType& X = _variables.at<0>()->estimate();
    float x               = geometry2d::t2v(X).x();
    float y               = geometry2d::t2v(X).y();
    _constraint(0, 0)     = x * x + y * y - _radius * _radius;
    if (constraint_only_) {
      return;
    }
    _J.setZero();
    _J(0, 0) = 2 * x;
    _J(1, 0) = 2 * y;
  }
  INSTANTIATE(SE2CircumferenceConstraintFactor)

} // namespace srrg2_solver
