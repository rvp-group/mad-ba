#include "sparse_block_linear_solver_cholesky.h"

namespace srrg2_solver {

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateStructure() {
    assert(_A && _b && " A matrix null");
    _L = SparseBlockCholesky(*_A);
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());
    _structure_changed = false;
    if (_L.choleskyAllocate()) {
      return SparseBlockLinearSolver::StructureGood;
    }
    return SparseBlockLinearSolver::StructureBad;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateCoefficients() {
    _L.setZero();
    _A->copyValues(_L);
    if (_L.choleskyUpdate()) {
      return SparseBlockLinearSolver::CoefficientsGood;
    }
    return SparseBlockLinearSolver::CoefficientsBad;
    _coefficients_changed = false;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateSolution() {
    _b->copyValues(_x);
    if (_L.choleskySolve(_x)) {
      return SparseBlockLinearSolver::SolutionGood;
    }
    return SparseBlockLinearSolver::SolutionBad;
  }

  struct IntPairColumnCompare{
    bool operator()(const IntPair& a, const IntPair& b) const {
      if (a.second<b.second) return true;
      if (a.second>b.second) return false;
      return a.first<b.first;
    }
  };

  bool
  SparseBlockLinearSolverCholesky::computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                                                       const std::vector<IntPair>& blocks_layout_) {
    using namespace std;
    std::vector<IntPair> blocks_layout=blocks_layout_;
    std::sort(blocks_layout.begin(),
              blocks_layout.end(),
              IntPairColumnCompare());
    
    // fill the matrix with block layout
    for (const IntPair& row_column_idx : blocks_layout) {
      MatrixBlockBase* block =
        inverse_blocks.blockAt(row_column_idx.second, row_column_idx.second, true);
      if (row_column_idx.first == row_column_idx.second) {
        block->setIdentity();
      } else {
        block->setZero();
      }
    }

    // do block column stuff
    for (size_t c=0; c<inverse_blocks._cols.size(); ++c){
      // solve linear system
      SparseBlockMatrix::IntBlockMap& col = inverse_blocks._cols[c];
      if (col.empty())
        continue;
      if (!_L.blockCholeskySolve(col)) {
        return false;
      }
      //delete non interesting blocks
      for (auto it=col.begin(); it!=col.end(); ++it) {
        int r=it->first;
        IntPair indices(r,c);
        if (!std::binary_search(blocks_layout.begin(),
                                blocks_layout.end(),
                                indices,
                                IntPairColumnCompare())){
          auto r_it=it;
          ++it;
          col.erase(r_it);
        }
      }
      
    }
    return true;
  }

} // namespace srrg2_solver
