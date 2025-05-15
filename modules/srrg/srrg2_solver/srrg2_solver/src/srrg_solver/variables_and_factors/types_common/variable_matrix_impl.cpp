#include "variable_matrix.h"
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver {
  template <int Rows_, int Cols_>
  void VariableMatrix_<Rows_, Cols_>::setZero() {
    this->_updated=true;
    this->_estimate.setZero();
  }

  template <int Rows_, int Cols_>
  void VariableMatrix_<Rows_, Cols_>::applyPerturbation(const typename VariableMatrix_<Rows_,Cols_>::PerturbationVectorType& pert) {
    this->_updated=true;
    int k=0;
    for (int r=0;  r<Rows_; ++r)
      for (int c=0; c<Cols_; ++c,++k)
        this->_estimate(r,c) += pert(k);
  }
  
} // namespace srrg2_solver
