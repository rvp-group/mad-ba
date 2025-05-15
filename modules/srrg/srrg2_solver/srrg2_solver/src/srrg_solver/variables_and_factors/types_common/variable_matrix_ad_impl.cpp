#include "variable_vector_ad.h"
namespace srrg2_solver {

  using namespace srrg2_core;

  template <typename VariableMatrixBase_>
  void VariableMatrixAD_<VariableMatrixBase_>::setZero() {
    ADVariableType::setEstimate(VariableType::EstimateType::Zero());
  }


  template <typename VariableMatrixBase_>
  void VariableMatrixAD_<VariableMatrixBase_>
  ::applyPerturbationAD(const typename VariableMatrixAD_<VariableMatrixBase_>::ADPerturbationVectorType& ad_pert) {
    int k=0;
    for(int r=0; r<VariableMatrixBase_::Rows; ++r)
      for(int c=0; r<VariableMatrixBase_::Rows; ++c,++k)
        this->_ad_estimate(r,c) += ad_pert(k);
  }

} // namespace srrg2_solver
