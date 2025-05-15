#pragma once
#include "ad_constraint_factor.h"
#include "ad_variable.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename ADConstraintFactorType, int idx>
  struct JUpdater {
    static inline void update(ADConstraintFactorType& adf) {
      JUpdater<ADConstraintFactorType, idx - 1>::update(adf);
      adf.template _updateJacobianAD<idx>();
    }
  };

  template <typename ADConstraintFactorType>
  struct JUpdater<ADConstraintFactorType, 0> {
    static inline void update(ADConstraintFactorType& adf) {
      adf.template _updateJacobianAD<0>();
    }
  };

  template <int ConstraintDim_, typename... VariableTypes_>
  void ADConstraintFactor_<ConstraintDim_, VariableTypes_...>::constraintAndJacobian(
    bool constraint_only) {
    Eigen::Matrix<DualValuef, ConstraintDim, 1> ad_constraint = this->operator()(this->variables());
    convertMatrix(BaseType::_constraint, ad_constraint);
    if (constraint_only) {
      return;
    }
    JUpdater<ThisType, NumVariables - 1>::update(*this);
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  template <int idx>
  void ADConstraintFactor_<ConstraintDim_, VariableTypes_...>::_updateJacobianAD() {
    const int PertDim       = BaseType::template perturbationDim<idx>();
    using ADVarPerturbation = Eigen::Matrix<DualValuef, PertDim, 1>;
    ADVarPerturbation pert;
    auto& var(this->variables().template at<idx>());
    // var fixed, derivative is zero;
    if (var->status() != VariableBase::Active) {
      return;
    }
    pert.setZero();
    var->setEstimate(var->estimate());
    int c_base = BaseType::template perturbationOffset<idx>();
    for (int c = 0; c < PertDim; ++c) {
      // we reset the variable derivatives
      pert(c, 0).derivative = 1.f;
      var->applyPerturbationAD(pert);
      Eigen::Matrix<DualValuef, ConstraintDim, 1> ad_constraint =
        this->operator()(this->variables());
      for (int r = 0; r < ConstraintDim; ++r) {
        BaseType::_J(r, c + c_base) = ad_constraint(r).derivative;
      }
      var->setEstimate(var->estimate());
      pert(c, 0).derivative = 0.f;
    }
  }

} // namespace srrg2_solver
