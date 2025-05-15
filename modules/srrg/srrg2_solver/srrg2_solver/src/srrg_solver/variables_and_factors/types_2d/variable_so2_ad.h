#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_so2.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE2 Pose AD Variable.
   * The template argument is the base Variable class
   */
  template <typename VariableSO2_>
  class VariableSO2AD_ : public ADVariable_<VariableSO2_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    using VariableType             = VariableSO2_;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    virtual ~VariableSO2AD_();
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert);
  };

  using VariableSO2RightAD = VariableSO2AD_<VariableSO2Right>;
  using VariableSO2LeftAD  = VariableSO2AD_<VariableSO2Left>;
} // namespace srrg2_solver
