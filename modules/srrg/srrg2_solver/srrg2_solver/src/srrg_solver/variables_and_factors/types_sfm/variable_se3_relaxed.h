#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  /** @brief SE3 Pose Variable base class.
   */
  class VariableSE3Relaxed : public Variable_<12, Isometry3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void applyPerturbation(const Vector6f& pert) override;
    void normalize() override;
};

} // namespace srrg2_solver
