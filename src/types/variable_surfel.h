#pragma once
#include <srrg_geometry/geometry_defs.h>
#include "srrg_solver/solver_core/variable.h"

namespace srrg2_solver {

  using namespace srrg2_core;
  class VariableSurfel : public Variable_<5, Isometry3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setZero() override;

    virtual void applyPerturbation(const Vector5f& pert) override;

    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  };

  class VariableSurfel1D : public Variable_<1, Isometry3_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setZero() override;

    virtual void applyPerturbation(const Vector1f& pert) override;

    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  };

}
