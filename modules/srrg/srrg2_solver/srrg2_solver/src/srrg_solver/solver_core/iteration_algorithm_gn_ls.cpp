#include "iteration_algorithm_gn_ls.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void IterationAlgorithmGNLS::setSolver(Solver* solver_) {
    IterationAlgorithmBase::setSolver(solver_);
    _gradient.clear();
    _direction.clear();
  }

  void IterationAlgorithmGNLS::_scaleVector(std::vector<float>& v, const float& scale) {
    int size = v.size();
    for (int i = 0; i < size; ++i) {
      v[i] *= scale;
    }
  }

  bool IterationAlgorithmGNLS::_lineSearch(IterationStats& istat) {
    float armijo_factor = 0;
    int size            = _direction.size();
    for (int i = 0; i < size; ++i) {
      armijo_factor += _gradient[i] * _direction[i];
    }
    float step_size = param_initial_step_size.value();
    updateChi(istat);
    float initial_chi        = istat.chi_kernelized;
    float predicted_decrease = 0;
    float chi                = std::numeric_limits<float>::max();
    std::cerr << "Initial chi : " << initial_chi << std::endl;
    while (step_size > 1e-8) {
      istat.num_internal_iteration++;
      push();
      _scaleVector(_direction, step_size);
      setPerturbation(_direction);
      applyPerturbation(istat);
      updateChi(istat);
      chi                = istat.chi_kernelized;
      predicted_decrease = initial_chi - step_size * param_beta.value() * armijo_factor;
      std::cerr << "Chi : " << chi << "\nPredicted decrease : " << predicted_decrease << std::endl;
      if (chi < predicted_decrease) {
        discardTop();
        istat.lambda = step_size;
        iterationStats().push_back(istat);
        return true;
      } else {
        float inv_step_size = 1.f / step_size;
        _scaleVector(_direction, inv_step_size);
        step_size *= param_tau.value();
        pop();
      }
    }
    return false;
  }

  bool IterationAlgorithmGNLS::_computeGaussNewtonStep(IterationStats& istat) {
    float damping = param_damping.value();
    std::vector<float> diagonal;
    getDiagonal(diagonal);
    for (size_t i = 0; i < diagonal.size(); ++i) {
      diagonal[i] += damping;
    }
    setDiagonal(diagonal);
    if (solveQuadraticForm(istat)) {
      getPerturbation(_direction);
      return true;
    }
    return false;
  }

  bool IterationAlgorithmGNLS::oneRound() {
    IterationStats istat;
    if (!buildQuadraticForm(istat)) {
      return false;
    }
    getRHS(_gradient);
    if (_computeGaussNewtonStep(istat)) {
      return _lineSearch(istat);
    } else {
      return false;
    }
  }
} // namespace srrg2_solver

