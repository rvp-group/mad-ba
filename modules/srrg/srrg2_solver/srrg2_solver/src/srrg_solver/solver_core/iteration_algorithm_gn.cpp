#include "iteration_algorithm_gn.h"
namespace srrg2_solver {
  using namespace srrg2_core;

  void IterationAlgorithmGN::setSolver(Solver* solver_) {
    IterationAlgorithmBase::setSolver(solver_);
    if (_solver) {
      getDiagonal(_diagonal);
      _diagonal_backup = _diagonal;
    }
  }

  bool IterationAlgorithmGN::oneRound() {
    // std::cerr << "one round" << std::endl;
    IterationStats istats;
    istats.reset();
    istats.iteration = currentIteration();
    // bool success=true;
    if (!buildQuadraticForm(istats)) {
      iterationStats().push_back(istats);
      // std::cerr << "QF fail" << std::endl;
      return false;
    }
    getDiagonal(_diagonal_backup);
    _diagonal=_diagonal_backup;
    if (param_damping.value() > 0) {
      istats.lambda = param_damping.value();
      for (size_t i = 0; i < _diagonal.size(); ++i) {
        _diagonal[i] += param_damping.value();
      }
      setDiagonal(_diagonal);
    }
    bool solve_ok=solveQuadraticForm(istats);
    setDiagonal(_diagonal_backup);
    if (solve_ok){
      applyPerturbation(istats);
    }
    iterationStats().push_back(istats);
    return solve_ok;
  }
} // namespace srrg2_solver
