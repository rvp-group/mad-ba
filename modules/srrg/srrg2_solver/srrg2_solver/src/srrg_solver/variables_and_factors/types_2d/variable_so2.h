#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry2d.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  class VariableSO2Base : public Variable_<1, Vector1_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum PerturbationSide { Left = 0x1, Right = 0x2 };
    virtual void setZero() override {
      setEstimate(Vector1f(0.f));
    }

  private:
  };

  template <VariableSO2Base::PerturbationSide PerturbationSide_ = VariableSO2Base::Right>
  class VariableSO2_ : public VariableSO2Base {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const VariableSO2Base::PerturbationSide PerturbationSide = PerturbationSide_;
    // break here the chain of indirection
    using BaseVariableType = VariableSO2_<PerturbationSide>;

    virtual void applyPerturbation(const Vector1f& pert) override {
      Matrix2f pert_m = geometry2d::a2r(pert(0));
      this->_updated  = true;
      float estimate  = _estimate(0);
      Matrix2f rot    = pert_m * geometry2d::a2r(estimate);
      switch (PerturbationSide) {
        case Left:
          _estimate = Vector1f(geometry2d::r2a(rot));
          break;
        case Right:
          rot       = geometry2d::a2r(estimate) * pert_m;
          _estimate = Vector1f(geometry2d::r2a(rot));
          break;
        default:
          assert(0);
      }
    }
  };

  using VariableSO2Right = VariableSO2_<VariableSO2Base::Right>;
  using VariableSO2Left  = VariableSO2_<VariableSO2Base::Left>;

} // namespace srrg2_solver
