#pragma once
#include "sparse_block_linear_solver.h"
#include <srrg_property/property_vector.h>

#include <Eigen/SparseQR>

namespace srrg2_solver {
  using namespace srrg2_core;
  // naive block solver that uses the cholesky block decomposition
  // no ordering is applied
  class SparseBlockLinearSolverNullspace : public SparseBlockLinearSolver {
  public:
    PARAM_VECTOR(PropertyVector_<float>,
                 nullspace_scales,
                 "scale for the i_th bullspace to apply to the solution",
                 0);

    // inverse would be a pseudo inverse, not computing here
    bool computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                             const std::vector<IntPair>& blocks_layout) override;

  protected:
    // computes the internal structure, given the structure of A
    virtual Status updateStructure();

    // copies the coefficients, and computes the numerical structure for the solver
    virtual Status updateCoefficients();

    // solves the linear system based on the updated coefficients
    virtual Status updateSolution();


    Eigen::SparseMatrix<double> _eigen_A;
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > _QR;
    Eigen::VectorXd _dense_x;
    std::vector <Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > _nullspace;
  };
} // namespace srrg2_solver
