#pragma once
#include "srrg_solver/solver_core/constraint_factor.h"
#include "variable_se2.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  class SE2CircumferenceConstraintFactor : public ConstraintFactor_<1, VariableSE2Right> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseFactorType = ConstraintFactor_<1, VariableSE2Right>;
    using EstimateType   = VariableSE2Right::EstimateType;

    void setRadius(const float& radius_) {
      _radius = radius_;
    }

    void constraintAndJacobian(bool constraint_only_ = false) final;

  protected:
    float _radius = 0.f;
  };
} // namespace srrg2_solver
