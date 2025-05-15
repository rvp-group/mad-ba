#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_vector.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  template <typename VariableMatrixBase_>
  class VariableMatrixAD_ : public ADVariable_<VariableMatrixBase_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VariableType             = VariableMatrixBase_;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    virtual ~VariableMatrixAD_() = default;

    virtual void setZero() override;

    virtual void applyPerturbationAD(const ADPerturbationVectorType& ad_pert);
    
  };

} // namespace srrg2_solver
