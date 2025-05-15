#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "variable_surfel.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE3 Pose AD Variable. The template argument is the
   * base Variable class
   */

  class VariableSurfelAD : public ADVariable_<VariableSurfel> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    using ADVariableType = ADVariable_<VariableSurfel> ;
    using VariableType   = VariableSurfel;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType ;
    using ADEstimateType = typename ADVariableType::ADEstimateType ;
    using Scalar = float;
    
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) {
      Vector6_<ad::DualValue_<Scalar>> pert6;
      for (int i=0; i<5; ++i)
        pert6(i)=pert(i);
      pert6(5)=ad::DualValue_<Scalar>(0.);
      ADVariableType::_ad_estimate = ADVariableType::_ad_estimate * geometry3d::v2t(pert6);
    }
  };

  class VariableSurfelAD1D : public ADVariable_<VariableSurfel1D> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*always replicate these typedefs. It's annoying but current compilers aren't smart enough.*/
    using ADVariableType = ADVariable_<VariableSurfel1D> ;
    using VariableType   = VariableSurfel1D;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType ;
    using ADEstimateType = typename ADVariableType::ADEstimateType ;
    using Scalar = float;
    
    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) {
      Vector1_<ad::DualValue_<Scalar>> pert1;
      pert1(0)=pert(0);
      ADVariableType::_ad_estimate.translation() += ADVariableType::_ad_estimate.linear().col(2) * pert1;
    }
  };

} // namespace srrg2_solver
