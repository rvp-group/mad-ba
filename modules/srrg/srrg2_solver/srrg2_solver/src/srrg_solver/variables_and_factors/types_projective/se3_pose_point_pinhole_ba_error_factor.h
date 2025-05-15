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
 
  class SE3PosePointPinholeBAErrorFactor : public ErrorFactor_<2,
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
    static bool computePrediction(Vector2f& point_in_image,
                           const VariableSE3QuaternionRight& pose,
                           const VariablePoint3& point,
                           const VariableMatrix3_4& proj,
                           const VariablePoint2& sizes);

    void errorAndJacobian(bool error_only_ = false) final;
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
    float z_cutoff=1e-2;
    bool inside_check=false;
  };
} // namespace srrg2_solver
