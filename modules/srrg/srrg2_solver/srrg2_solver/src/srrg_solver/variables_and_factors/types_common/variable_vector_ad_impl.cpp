#pragma once
#include "variable_vector_ad.h"
namespace srrg2_solver {

  using namespace srrg2_core;

  template <typename VariableVectorBase_>
  void VariableVectorAD_<VariableVectorBase_>::setZero() {
    ADVariableType::setEstimate(VariableType::EstimateType::Zero());
  }


  template <typename VariableVectorBase_>
  void VariableVectorAD_<VariableVectorBase_>
  ::applyPerturbationAD(const typename VariableVectorAD_<VariableVectorBase_>::ADPerturbationVectorType& ad_pert) {
    this->_ad_estimate += ad_pert;
  }

} // namespace srrg2_solver
