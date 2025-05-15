#pragma once
#include "iteration_algorithm_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class IterationAlgorithmGNLS : public IterationAlgorithmBase {
  public:
    PARAM(PropertyFloat,
          initial_step_size,
          "Initial step size for the Armijo line search",
          1.f,
          nullptr);
    PARAM(PropertyFloat, beta, "beta for the armijo line search", 0.0001f, nullptr);
    PARAM(PropertyFloat, tau, "discout factor for the step size during line seach", 0.7f, nullptr);
    PARAM(PropertyFloat, damping, "Damping of the GN step ( H + damping * I )", 1e-8, 0);
    void setSolver(Solver* solver_) final;
    bool oneRound() final;

  protected:
    bool _computeGaussNewtonStep(IterationStats& istat);
    void _scaleVector(std::vector<float>& v, const float& scale);
    bool _lineSearch(IterationStats& istat);
    std::vector<float> _gradient;
    std::vector<float> _direction;
  };

} // namespace srrg2_solver_hierarchical

