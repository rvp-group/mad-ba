#include "sparse_block_linear_solver_cholesky_diagonal.h"
#include "quotient_graph.h"

namespace srrg2_solver {
  void SparseBlockLinearSolverCholeskyDiagonal::computeOrderingHint(
    std::vector<int>& ordering,
    const std::vector<IntPair>& layout) const {
    const int dim = ordering.size();
    for (int i = 0; i < dim; ++i) {
      ordering[i] = i;
    }
  }

} // namespace srrg2_solver
