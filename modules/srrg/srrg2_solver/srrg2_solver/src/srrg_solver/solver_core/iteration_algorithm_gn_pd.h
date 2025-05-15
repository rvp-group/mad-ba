#pragma once
#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class IterationAlgorithmGNPD : public IterationAlgorithmBase {
  public:
    PARAM(PropertyInt, num_runs, "Number of internal iterations", 100, nullptr);
    PARAM(PropertyFloat, damping, "Damping of the GN step ( H + damping * I )", 10, 0);
    void setSolver(Solver* solver_) final;
    bool oneRound() final;

  protected:
    bool _computeGaussNewtonStep(IterationStats& istat);
    bool _primalDual(IterationStats& istat);
    std::vector<float> _direction;
  };

} // namespace srrg2_solver
