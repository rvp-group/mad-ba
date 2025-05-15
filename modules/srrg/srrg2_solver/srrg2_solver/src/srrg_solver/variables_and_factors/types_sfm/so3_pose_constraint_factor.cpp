#include "so3_pose_constraint_factor.h"
#include "srrg_solver/solver_core/constraint_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {

  using namespace std;

  void SO3RelaxedPoseConstraintFactor::constraintAndJacobian(bool constraint_only_) {
    // bb take objects
    auto* v       = _variables.at<0>();
    const auto R  = v->estimate();
    const auto r1 = R.block<1, 3>(0, 0);
    const auto r2 = R.block<1, 3>(1, 0);
    const auto r3 = R.block<1, 3>(2, 0);
    Eigen::Matrix<float, 9, 1> I_flat;
    I_flat.setZero();
    I_flat(0) = 1;
    I_flat(4) = 1;
    I_flat(8) = 1;

    _constraint = srrg2_solver::VariableSO3Relaxed::flatten(R * R.transpose()) - I_flat;
    if (constraint_only_) {
      return;
    }

    Eigen::Matrix<float, 9, 9> Jm;
    Jm.setZero();
    Jm.block<3, 3>(0, 0) = R;
    Jm.block<3, 3>(3, 3) = R;
    Jm.block<3, 3>(6, 6) = R;
    Jm.block<1, 3>(0, 0) += r1;
    Jm.block<1, 3>(1, 3) += r1;
    Jm.block<1, 3>(2, 6) += r1;
    Jm.block<1, 3>(3, 0) += r2;
    Jm.block<1, 3>(4, 3) += r2;
    Jm.block<1, 3>(5, 6) += r2;
    Jm.block<1, 3>(6, 0) += r3;
    Jm.block<1, 3>(7, 3) += r3;
    Jm.block<1, 3>(8, 6) += r3;
    jacobian<0>() = Jm;
  }

  INSTANTIATE(SO3RelaxedPoseConstraintFactorAD)

} // namespace srrg2_solver
