#include "variable_vector.h"

namespace srrg2_solver {
  template <int Dim_>
  void VariableVector_<Dim_>::setZero() {
    this->_updated = true;
    this->_estimate.setZero();
  }

  template <int Dim_>
  void VariableVector_<Dim_>::applyPerturbation(
    const typename VariableVector_<Dim_>::EstimateType& pert) {
    this->_updated = true;
    this->_estimate += pert;
  }

} // namespace srrg2_solver
