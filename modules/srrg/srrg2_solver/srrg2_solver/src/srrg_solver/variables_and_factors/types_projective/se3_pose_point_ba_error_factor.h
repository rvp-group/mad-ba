#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/variables_and_factors/types_common/all_types.h" //point2, point3 and projection matrix
#include "srrg_solver/variables_and_factors/types_3d/variable_se3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /**
   * @brief Bundle adjustment + calibartion 3D factor for Pose-Point-Offset estimation.
   * Error is computed by referring the point in the world frame (inverse pose of the sensor in
   * world) and computing the difference with the measure
   */
 
  class SE3PosePointBAErrorFactor : public ErrorFactor_<2,
                                                        // robot pose
                                                        VariableSE3QuaternionRight,
                                                        // point in world
                                                        VariablePoint3,
                                                        // const! projection matrix KR | Kt
                                                        VariableMatrix3_4,
                                                        // const! image size (rows, cols)
                                                        VariablePoint2
                                                        >,
                                        public MeasurementOwnerEigen_<Vector2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
} // namespace srrg2_solver
