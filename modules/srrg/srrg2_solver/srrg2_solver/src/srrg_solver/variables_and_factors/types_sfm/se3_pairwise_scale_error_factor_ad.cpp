#include "se3_pairwise_scale_error_factor_ad.h"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  template <typename  VariableType_>
  typename SE3PairwiseScaleErrorFactorAD_<VariableType_>::ADErrorVectorType
  SE3PairwiseScaleErrorFactorAD_<VariableType_>::operator()
    (typename SE3PairwiseScaleErrorFactorAD_<VariableType_>::VariableTupleType& vars) {
    const Isometry3_<DualValuef>& from = vars. template at<0>()->adEstimate();
    const Isometry3_<DualValuef>& to   = vars. template at<1>()->adEstimate();
    Vector3_<DualValuef> tij = (from.inverse() * to).translation();
    DualValuef squared_norm=tij.squaredNorm();
    ADErrorVectorType err;
    err[0]=log(squared_norm/_squared_range);
    return err;
  }

  INSTANTIATE(SE3PairwiseScaleErrorFactorEulerLeftAD);
  INSTANTIATE(SE3PairwiseScaleErrorFactorEulerRightAD);
  INSTANTIATE(SE3PairwiseScaleErrorFactorQuaternionLeftAD);
  INSTANTIATE(SE3PairwiseScaleErrorFactorQuaternionRightAD);
  
}
