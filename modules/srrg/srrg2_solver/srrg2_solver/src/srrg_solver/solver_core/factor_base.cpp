#include "factor_base.h"
#include "robustifier.h"

namespace srrg2_solver {

  bool FactorBase::robustify() {
    _stats.status     = FactorStats::Status::Inlier;
    _stats.kernel_chi = _stats.chi;
    _kernel_scales << _stats.chi, 1., 0;
    if (!_robustifier) {
      return false;
    }
    bool robustified  = _robustifier->robustify(_kernel_scales, _stats.chi);
    _stats.kernel_chi = _kernel_scales[0];
    if (robustified) {
      _stats.status = FactorStats::Status::Kernelized;
    }
    return true;
  }

  void FactorBase::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("FactorBase::draw|invalid canvas");
    std::cerr << "FactorBase::draw not implemented" << std::endl;
  }

  void FactorBase::resetStats() {
    _stats.status               = FactorStats::Suppressed;
    _stats.chi                  = 0;
    _stats.kernel_chi           = 0;
    _stats.constraint_violation = -1;
  }

} // namespace srrg2_solver
