#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "variable_se3.h"

namespace srrg2_solver {
  /**
   * @brief 3D PGO error factor exploiting chordal distance.
   * Error vector is 12 component (from h(x) - Z)
   * vertices use EULER angles with increment POST-multiplied
   */
  class SE3PosePoseChordalEulerRightErrorFactor
    : public ErrorFactor_<12, VariableSE3EulerRight, VariableSE3EulerRight>,
      public MeasurementOwnerEigen_<Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<12, VariableSE3EulerRight, VariableSE3EulerRight>;
    using Scalar   = MeasurementType::Scalar;
    void errorAndJacobian(bool error_only_ = false);

    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
} // namespace srrg2_solver
