#include "iteration_algorithm_gn_pd.h"

namespace srrg2_solver {

  void IterationAlgorithmGNPD::setSolver(Solver* solver_) {
    IterationAlgorithmBase::setSolver(solver_);
  }

  bool IterationAlgorithmGNPD::_primalDual(IterationStats& istats) {
    std::cerr << param_num_runs.value() << std::endl;
    for (int i = 0; i < param_num_runs.value(); ++i) {
      // bb save estimate on stack
      push();
      istats.num_internal_iteration = i;
      std::cerr << "num internal iteration: " << istats.num_internal_iteration << std::endl;
      // bb update Q and b
      // bb will always be the same for all factors other than constraint ones
      if (!buildQuadraticForm(istats)) {
        iterationStats().push_back(istats);
        std::cerr << "fail BQ" << std::endl;
        return false;
      }
      _computeGaussNewtonStep(istats);
      // bb change status of the variables
      // bb update the last perturbation
      applyPerturbation(istats);
      if (i < param_num_runs.value() - 1) {
        // bb retrieve estimate from stack
        pop();
      }
    }
    discardTop();
    iterationStats().push_back(istats);
    return true;
  }

  bool IterationAlgorithmGNPD::_computeGaussNewtonStep(IterationStats& istat) {
    float damping = param_damping.value();
    std::vector<float> diagonal;
    getDiagonal(diagonal);
    for (size_t i = 0; i < diagonal.size(); ++i) {
      diagonal[i] += damping;
    }
    setDiagonal(diagonal);
    if (solveQuadraticForm(istat)) {
      return true;
    }
    return false;
  }

  bool IterationAlgorithmGNPD::oneRound() {
    IterationStats istats;
    istats.reset();
    istats.iteration = currentIteration();
    return _primalDual(istats);
  }
} // namespace srrg2_solver
