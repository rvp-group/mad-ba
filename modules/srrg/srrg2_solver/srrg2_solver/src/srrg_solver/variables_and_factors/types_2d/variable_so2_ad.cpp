#include "variable_so2_ad.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  using namespace srrg2_core;

  template <typename VariableSO2_>
  VariableSO2AD_<VariableSO2_>::~VariableSO2AD_() {
  }

  template <typename VariableSO2_>
  void VariableSO2AD_<VariableSO2_>::applyPerturbationAD(const ADPerturbationVectorType& pert) {
    Matrix2_<DualValuef> pert_m;
    pert_m                   = geometry2d::a2r(pert(0));
    DualValuef estimate      = ADVariableType::_ad_estimate(0);
    Matrix2_<DualValuef> rot = pert_m * geometry2d::a2r(estimate);
    switch (VariableType::PerturbationSide) {
      case VariableSO2Base::Left:
        ADVariableType::_ad_estimate = Vector1_<DualValuef>(geometry2d::r2a(rot));
        break;
      case VariableSO2Base::Right:
        rot                          = geometry2d::a2r(estimate) * pert_m;
        ADVariableType::_ad_estimate = Vector1_<DualValuef>(geometry2d::r2a(rot));
        break;
      default:
        assert(0);
    }
  }

  INSTANTIATE(VariableSO2RightAD)
  INSTANTIATE(VariableSO2LeftAD)

} // namespace srrg2_solver
