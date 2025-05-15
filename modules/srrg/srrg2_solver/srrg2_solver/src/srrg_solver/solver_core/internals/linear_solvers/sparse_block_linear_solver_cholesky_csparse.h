#pragma once
#include "sparse_block_linear_solver_cholesky.h"

namespace srrg2_solver {

  class SparseBlockLinearSolverCholeskyCSparse : public SparseBlockLinearSolverCholesky {
  public:
    virtual void computeOrderingHint(std::vector<int>& ordering,
                                     const std::vector<IntPair>& block_layout) const;
  };
  using SparseBlockLinearSolverCholeskyCSparsePtr =
    std::shared_ptr<SparseBlockLinearSolverCholeskyCSparse>;

} // namespace srrg2_solver
