#pragma once
#include "imu_bias_variable.h"
#include "preintegrated_imu_measurements.h"
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_common/all_types.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver {

  using namespace srrg2_core;
  //! @brief pose pose error factor ad that uses quaternion vertices
  class ImuErrorFactorAD : public ADErrorFactor_<15,
                                                 VariableSE3QuaternionRightAD,
                                                 VariableVector3AD,
                                                 VariableSE3QuaternionRightAD,
                                                 VariableVector3AD,
                                                 VariableImuBiasAD,
                                                 VariableImuBiasAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ADErrorFactor_<15,
                                    VariableSE3QuaternionRightAD,
                                    VariableVector3AD,
                                    VariableSE3QuaternionRightAD,
                                    VariableVector3AD,
                                    VariableImuBiasAD,
                                    VariableImuBiasAD>;

    ImuErrorFactorAD() {
      Vector3f gravity(0.f, 0.f, -9.80655);
      convertMatrix(_g, gravity);
    }

    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;

    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override;
    
    //! @brief converts the measurement in dual values
    // ldg convert each damn matrix
    void setMeasurement(const PreintegratedImuMeasurements& meas_);
    
    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    //! @brief measurement
    Matrix3_<DualValuef> _R_meas_biased;
    Vector3_<DualValuef> _p_meas_biased;
    Vector3_<DualValuef> _v_meas_biased;
    Vector3_<DualValuef> _bias_acc_hat;
    Vector3_<DualValuef> _bias_omega_hat;
    DualValuef _delta_t;
    // ldg jacobians for bias correction
    Matrix3_<DualValuef> _dRij_dbg;
    Matrix3_<DualValuef> _dpij_dba;
    Matrix3_<DualValuef> _dpij_dbg;
    Matrix3_<DualValuef> _dvij_dba;
    Matrix3_<DualValuef> _dvij_dbg;
    Vector3_<DualValuef> _g;
  };

} // namespace srrg2_solver
