#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>
#include "variable_surfel_ad.h"

namespace srrg2_solver {
  using namespace srrg2_core;
   class SE3PoseSurfelQuaternionErrorFactorAD
    : public ADErrorFactor_<1, VariableSE3QuaternionRightAD,
                            VariableSurfelAD>,
      public MeasurementOwnerEigen_<Isometry3f> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseFunctorType = ADErrorFactor_<1, VariableSE3QuaternionRightAD,
                            VariableSurfelAD>;
     
    using VariableTupleType = typename BaseFunctorType::VariableTupleType;

    //! @brief compute the error as quaternion
    BaseFunctorType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      // pose
      const Isometry3_<DualValuef>& from = vars.at<0>()->adEstimate();
      // surfel in map
      const Isometry3_<DualValuef>& to   = vars.at<1>()->adEstimate();
      
      Isometry3_<DualValuef> prediction = from.inverse() * to;
      Vector3_<DualValuef> prediction_point=prediction.translation();
      Vector3_<DualValuef> prediction_normal=prediction.linear().col(2);
      Vector3_<DualValuef> measured_point=_ad_measurement.translation();
      //Vector3_<DualValuef> to_normal=_ad_measurement.linear().col(2);
      BaseFunctorType::ADErrorVectorType error;
      error(0,0)= prediction_normal.dot(prediction_point-measured_point);
      return error;
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Isometry3f& iso) override {
      _measurement = iso;
      convertMatrix(_ad_measurement, iso);
    }
    
    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    // here we store the measurement
    Isometry3_<DualValuef> _ad_measurement = Isometry3_<DualValuef>::Identity();
     
   };

   class SE3PoseSurfelQuaternionErrorFactorAD1D
    : public ADErrorFactor_<1, VariableSE3QuaternionRightAD,
                            VariableSurfelAD1D>,
      public MeasurementOwnerEigen_<Isometry3f> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseFunctorType = ADErrorFactor_<1, VariableSE3QuaternionRightAD,
                            VariableSurfelAD1D>;
     
    using VariableTupleType = typename BaseFunctorType::VariableTupleType;

    //! @brief compute the error as quaternion
    BaseFunctorType::ADErrorVectorType operator()(VariableTupleType& vars) final {
      // pose
      const Isometry3_<DualValuef>& from = vars.at<0>()->adEstimate();
      // surfel in map
      const Isometry3_<DualValuef>& to   = vars.at<1>()->adEstimate();
      
      Isometry3_<DualValuef> prediction = from.inverse() * to;
      Vector3_<DualValuef> prediction_point=prediction.translation();
      Vector3_<DualValuef> prediction_normal=prediction.linear().col(2);
      Vector3_<DualValuef> measured_point=_ad_measurement.translation();
      //Vector3_<DualValuef> to_normal=_ad_measurement.linear().col(2);
      BaseFunctorType::ADErrorVectorType error;
      error(0,0)= prediction_normal.dot(prediction_point-measured_point);
      return error;
    }

    // define this to set the measurement and make the error function ready
    // this will be visible in the factor!
    void setMeasurement(const Isometry3f& iso) override {
      _measurement = iso;
      convertMatrix(_ad_measurement, iso);
    }
    
    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    // here we store the measurement
    Isometry3_<DualValuef> _ad_measurement = Isometry3_<DualValuef>::Identity();
     
   };
}
