#pragma once
#include "srrg_solver/solver_core/ad_variable.h"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief  VariableImuBias Variable.
   */
  class VariableImuBias : public Variable_<6, Vector6_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseVariableType = VariableImuBias;

    virtual ~VariableImuBias() = default;
    void setZero();

    inline void setBiasAccelerometerEstimate(const Vector3f& ba) {
      _estimate.tail<3>() = ba;
    }

    inline void setBiasGyroscopeEstimate(const Vector3f& bg) {
      _estimate.head<3>() = bg;
    }

    inline Vector3f biasAccelerometer() const {
      return _estimate.tail<3>();
    }

    inline Vector3f biasGyroscope() const {
      return _estimate.head<3>();
    }

    void applyPerturbation(const Vector6f& pert) override;

    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };

  class VariableImuBiasAD : public ADVariable_<VariableImuBias> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VariableType             = VariableImuBias;
    using ADVariableType           = ADVariable_<VariableType>;
    using ADPerturbationVectorType = typename ADVariableType::ADPerturbationVectorType;
    using ADEstimateType           = typename ADVariableType::ADEstimateType;

    ~VariableImuBiasAD() = default;
    void applyPerturbationAD(const ADPerturbationVectorType& ad_pert);
  };
} // namespace srrg2_solver
