#pragma once
#include "srrg_solver/solver_core/error_factor.h"
#include "srrg_solver/variables_and_factors/types_common/all_types.h" //point2, point3 and projection matrix
#include "srrg_solver/variables_and_factors/types_3d/variable_se3.h"

namespace srrg2_solver {
  using namespace srrg2_core;


  class SE3PosePointRectifiedStereoErrorFactor : public ErrorFactor_<3, // measurement col_left, row_left, disparity (col_left-col_right)
                                                        // robot pose
                                                        VariableSE3QuaternionRight,
                                                        // point in world
                                                        VariablePoint3,
                                                        // const! projection matrix KR | Kt 
                                                        VariableMatrix3_4,
                                                        // const! rows cols baseline_in_pixels
                                                        VariablePoint3
                                                        >,
                                        public MeasurementOwnerEigen_<Vector3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static bool computePrediction(Vector3f& point_in_image,
                           const VariableSE3QuaternionRight& pose,
                           const VariablePoint3& point,
                           const VariableMatrix3_4& proj,
                           const VariablePoint3& sizes_and_baseline);
    
    static bool triangulate(Vector3f& point_in_world,
                     const Vector3f& measurement_,
                     const VariableSE3QuaternionRight& pose,
                     const VariableMatrix3_4& proj,
                     const VariablePoint3& sizes_and_baseline);
    
    void errorAndJacobian(bool error_only_ = false);
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
    float z_cutoff=1e-2;
  };
} // namespace srrg2_solver
