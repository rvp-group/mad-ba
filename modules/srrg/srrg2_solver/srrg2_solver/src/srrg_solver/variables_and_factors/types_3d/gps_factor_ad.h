#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver {

  using namespace srrg2_core;
  //! @brief pose pose error factor ad that uses quaternion vertices
  class GpsErrorFactorAD : public ADErrorFactor_<3, VariableSE3QuaternionRightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<3, VariableSE3QuaternionRightAD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override {
      ADErrorVectorType error = vars.at<0>()->adEstimate().translation() - _pose_measurement;
      return error;
    }

    //! @brief converts the measurement in dual values
    void setMeasurement(const Vector3f& gps_measurement_) {
      convertMatrix(_pose_measurement, gps_measurement_);
    }

    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    //! @brief measurement
    Vector3_<DualValuef> _pose_measurement;
  };

} // namespace srrg2_solver