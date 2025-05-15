#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver {

  //! @brief pose factor to impose the scale
  template <typename VariableType_>
  class SE3PairwiseScaleErrorFactorAD_
    : public ADErrorFactor_<1, VariableType_, VariableType_>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType =
      ADErrorFactor_<1, VariableType_, VariableType_>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override ;
    void setRange(const float& range) {
      _squared_range.value= range*range;
    }
  protected:
    DualValuef _squared_range=DualValuef(1.,0.);
  };

  using SE3PairwiseScaleErrorFactorEulerLeftAD = SE3PairwiseScaleErrorFactorAD_<VariableSE3EulerLeftAD>;
  using SE3PairwiseScaleErrorFactorEulerRightAD = SE3PairwiseScaleErrorFactorAD_<VariableSE3EulerRightAD>;
  using SE3PairwiseScaleErrorFactorQuaternionLeftAD = SE3PairwiseScaleErrorFactorAD_<VariableSE3QuaternionLeftAD>;
  using SE3PairwiseScaleErrorFactorQuaternionRightAD = SE3PairwiseScaleErrorFactorAD_<VariableSE3QuaternionRightAD>;

  using SE3PairwiseScaleErrorFactorEulerLeftADPtr = std::shared_ptr<SE3PairwiseScaleErrorFactorEulerLeftAD>;
  using SE3PairwiseScaleErrorFactorEulerRightADPtr = std::shared_ptr<SE3PairwiseScaleErrorFactorEulerRightAD>;
  using SE3PairwiseScaleErrorFactorQuaternionLeftADPtr = std::shared_ptr<SE3PairwiseScaleErrorFactorQuaternionLeftAD>;
  using SE3PairwiseScaleErrorFactorQuaternionRightADPtr = std::shared_ptr<SE3PairwiseScaleErrorFactorQuaternionRightAD>;
}
