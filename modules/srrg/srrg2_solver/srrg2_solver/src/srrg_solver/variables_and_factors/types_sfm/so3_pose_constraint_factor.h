#pragma once
#include "variable_so3.h"
#include <srrg_solver/solver_core/ad_constraint_factor.h>
#include <srrg_solver/solver_core/ad_constraint_factor_impl.cpp>
#include <srrg_solver/solver_core/constraint_factor.h>

namespace srrg2_solver {

  class SO3RelaxedPoseConstraintFactor : public ConstraintFactor_<9, VariableSO3Relaxed> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ConstraintFactor_<9, VariableSO3Relaxed>;
    void constraintAndJacobian(bool constraint_only_ = false) override;

  protected:
  };

  class SO3RelaxedPoseConstraintFactorAD : public ADConstraintFactor_<9, VariableSO3RelaxedAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType               = ADConstraintFactor_<9, VariableSO3RelaxedAD>;
    using VariableTupleType      = typename BaseType::VariableTupleType;
    using ADConstraintVectorType = typename BaseType::ADConstraintVectorType;
    ADConstraintVectorType operator()(VariableTupleType& vars) final {
      ADConstraintVectorType constraint = ADConstraintVectorType::Zero();
      const Matrix3_<DualValuef>& R     = vars.at<0>()->adEstimate();

      constraint =
        VariableSO3RelaxedAD::flatten(R * R.transpose() - Matrix3_<DualValuef>::Identity());
      return constraint;
    }

  protected:
  };

} // namespace srrg2_solver
