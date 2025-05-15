#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  template <int PerturbationDim_>
  class VariableSO3_ : public Variable_<PerturbationDim_, Matrix3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType       = Variable_<9, Matrix3_>;
    using PerturbationVectorType = BaseVariableType::PerturbationVectorType;
    void setZero() override {
      this->_estimate.setIdentity();
    }
  };

  /** @brief SE3 Pose Variable base class.
   */
  class VariableSO3Relaxed : public VariableSO3_<9> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType       = Variable_<9, Matrix3_>;
    using PerturbationVectorType = BaseVariableType::PerturbationVectorType;
    void applyPerturbation(const PerturbationVectorType& pert) override;
    void normalize() override;

    inline static PerturbationVectorType flatten(const Matrix3f& m) {
      PerturbationVectorType v;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          v(r * 3 + c) = m(r, c);
      return v;
    }

    inline static Matrix3f unflatten(const PerturbationVectorType& v) {
      Matrix3f m;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          m(r, c) = v(r * 3 + c);
      return m;
    }
  };

  class VariableSO3RelaxedAD : public ADVariable_<VariableSO3Relaxed> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ADVariableType           = ADVariable_<VariableSO3Relaxed>;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;
    using VariableType             = VariableSO3Relaxed;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;

    inline static Matrix3_<DualValuef> unflatten(const ADPerturbationVectorType& v) {
      Matrix3_<DualValuef> m;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          m(r, c) = v(r * 3 + c);
      return m;
    }

    inline static ADPerturbationVectorType flatten(const Matrix3_<DualValuef>& m) {
      ADPerturbationVectorType v;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          v(r * 3 + c) = m(r, c);
      return v;
    }

    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) {
      ADEstimateType est           = ADVariableType::_ad_estimate;
      ADEstimateType pert_m        = unflatten(pert);
      ADVariableType::_ad_estimate = pert_m + est;
    }
  };

  class VariableSO3QuaternionRight : public VariableSO3_<3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType       = Variable_<3, Matrix3_>;
    using PerturbationVectorType = BaseVariableType::PerturbationVectorType;
    void applyPerturbation(const PerturbationVectorType& pert) override {
      this->_estimate = this->_estimate * geometry3d::nq2r(pert);
    }
  };

  class VariableSO3QuaternionRightAD : public ADVariable_<VariableSO3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ADVariableType           = ADVariable_<VariableSO3QuaternionRight>;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;
    using VariableType             = VariableSO3QuaternionRight;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;

    virtual void applyPerturbationAD(const ADPerturbationVectorType& pert) {
      ADEstimateType pert_m        = geometry3d::nq2r(pert);
      ADVariableType::_ad_estimate = ADVariableType::_ad_estimate * pert_m;
    }
  };

} // namespace srrg2_solver
