#pragma once

#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/solver_core/ad_error_factor.h"
#include "variable_so3.h"

namespace srrg2_solver {

  /**
   * @brief Standard SE3 PGO factor.
   * Error vector is composed like [x y z qx qy qz]
   */
  class SO3RelaxedPosePoseErrorFactor
    : public ErrorFactor_<9, VariableSO3Relaxed, VariableSO3Relaxed>,
      public MeasurementOwnerEigen_<Matrix3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<9, VariableSO3Relaxed, VariableSO3Relaxed>;
    using Scalar   = MeasurementType::Scalar;
    using MeasurementOwnerType = MeasurementOwnerEigen_<Matrix3f>;
    void errorAndJacobian(bool error_only_ = false) override;

  protected:
  };

  class SO3PosePoseQuaternionRightErrorFactorAD:
    public ADErrorFactor_<3, VariableSO3QuaternionRightAD, VariableSO3QuaternionRightAD>,
    public MeasurementOwnerEigen_<Matrix3f>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<3, VariableSO3QuaternionRightAD, VariableSO3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;

    ADErrorVectorType operator()(VariableTupleType& vars) override;

    inline void setMeasurement(const Matrix3f& measurement_) override {
      _measurement = measurement_;
      convertMatrix(_ad_inverse_measurement, _measurement.transpose());
    }
  protected:
    Matrix3_<DualValuef> _ad_inverse_measurement;

  };
} // namespace srrg2_solver
