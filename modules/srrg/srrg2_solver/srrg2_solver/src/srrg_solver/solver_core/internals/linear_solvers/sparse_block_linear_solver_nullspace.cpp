#include "sparse_block_linear_solver_nullspace.h"

namespace srrg2_solver {
  using namespace std;
  SparseBlockLinearSolver::Status SparseBlockLinearSolverNullspace::updateStructure() {
    assert(_A && _b && " A matrix null");
    cerr << "QR structure| filling eigen...";
    _A->fillEigenSparse(_eigen_A,true);
    cerr << "DONE" << endl;
    _eigen_A.makeCompressed();
    cerr << "QR structure| analyzing pattern... " << endl;
    _QR.analyzePattern(_eigen_A);
    cerr << "done" << endl;
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());
    _structure_changed = false;
    return SparseBlockLinearSolver::StructureGood;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverNullspace::updateCoefficients() { 
    cerr << "QR update| filling eigen ...";
    _eigen_A.setZero();
    _A->fillEigenSparse(_eigen_A,true);
    cerr << "QR update| compressing eigen ...";
    _eigen_A.makeCompressed();
    cerr << "DONE " << endl;
    cerr << "QR update| factorizing... ";
    _QR.factorize(_eigen_A);
    cerr << "done" << endl;
    
    cerr << "QR update| computing nullspace ..." << endl;
    Eigen::VectorXd v(_eigen_A.cols());
    v.setZero();
    cerr << "RANK: " << _QR.rank() << endl;
    _nullspace.resize(param_nullspace_scales.size());
    for (int r=_eigen_A.cols()-1; r>=_eigen_A.cols()-(int)param_nullspace_scales.size(); --r) {
      //cerr << "r:" <<r << endl;
      v(r)=1;

      Eigen::VectorXd n=_QR.matrixQ()*v;
      float gamma=(_eigen_A*n).squaredNorm();
      cerr << "\tnullspace[" << _eigen_A.cols()-r-1 << "]: " << "norm(_eigen_A*n): " << gamma << endl;
      // std::string filename=std::string("ns_")+std::to_string(A.cols()-r)+std::string(".dat");
      // cerr << "filename: [" << filename << "]" << endl;
      // ofstream os(filename);
      // for (int i=0; i<n.rows(); ++i) {
      //   if (! (i%3))
      //     os << endl;
      //   os << n[i]*100 << " ";
      _nullspace[_eigen_A.cols()-r-1]=n;
      v(r)=0;
    }
    cerr << "QR update| computing nullspace DONE " << endl;
    
    cerr << "QR update| coefficients: getting" << endl;
    std::vector<float> dense_x;
    _b->getDenseVector(dense_x);
    cerr << "QR update| coefficients: mapping" << endl;
    Eigen::Map<Eigen::VectorXf> dense_x_map(&dense_x[0],dense_x.size(),1);
    cerr << "QR update| coefficients: casting" << endl;
    _dense_x=dense_x_map.cast<double>();
    _coefficients_changed = false;
    return SparseBlockLinearSolver::CoefficientsGood;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverNullspace::updateSolution() {
    cerr << "QR solve| compute solution" << endl;
    _dense_x = _QR.solve(_dense_x);
    //_dense_x.setZero();
    for (size_t i=0; i<_nullspace.size(); ++i) {
      _dense_x += _nullspace[i]*param_nullspace_scales.value(i);
    }
    cerr << "QR solve| solution norm: " << (_eigen_A*_dense_x).squaredNorm();
    cerr << "QR solve| casting double->float" << endl;
    std::vector<float> dense_x(_dense_x.rows());
    Eigen::Map<Eigen::VectorXf> dense_x_map(&dense_x[0],dense_x.size(),1);
    dense_x_map=_dense_x.cast<float>();
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());
    _x.allocateFull();
    cerr << "QR solve| copy back" << endl;
    cerr << "src size: " << dense_x.size()  << " dest_size: " << _x.rows() << endl;
    _x.fromDenseVector(dense_x);
    return SparseBlockLinearSolver::SolutionGood;
  }

  bool
  SparseBlockLinearSolverNullspace::computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                                                       const std::vector<IntPair>& blocks_layout_) {
    throw std::runtime_error ("SparseBlockLinearSolverNullspace::computeBlockInverse| not yet implemented");
    return true;
  }

} // namespace srrg2_solver
