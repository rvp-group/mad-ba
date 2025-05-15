#pragma once
#include <srrg_solver/solver_core/variable.h>

#include <srrg_geometry/geometry3d.h>
#include <srrg_geometry/similiarity.hpp>

namespace srrg2_solver {
  using namespace srrg2_core;

  class VariableSim3Base : public VariableGeneric_<7, Similiarity3_> {
  public:
    enum PerturbationSide { Left = 0x1, Right = 0x2 };
    enum PerturbationType { Quaternion = 0x1, Euler = 0x2 };
    using EstimateType = VariableGeneric_<7, Similiarity3_>::EstimateType;

    void setZero() override;
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;
    void normalize() override;
  };

  
  template <VariableSim3Base::PerturbationType PerturbationType_ = VariableSim3Base::Euler,
            VariableSim3Base::PerturbationSide PerturbationSide_ = VariableSim3Base::Right>
  class VariableSim3_ : public VariableSim3Base {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const VariableSim3Base::PerturbationSide PerturbationSide = PerturbationSide_;
    static const VariableSim3Base::PerturbationType PerturbationType = PerturbationType_;
    using BaseVariableType = VariableSim3_<PerturbationType_, PerturbationSide_>;

    void applyPerturbation(const Vector7f& pert) override {
      this->_updated = true;
      typedef VariableSim3Base BaseT;
      BaseT::EstimateType pert_m;
      switch (PerturbationType) {
        case Euler:
          pert_m = geometry3d::tas2s(pert);
          break;
        case Quaternion:
          pert_m = geometry3d::v2s(pert);
      }

      switch (PerturbationSide) {
        case Left:
          _estimate = pert_m * _estimate;
          break;
        case Right:
          _estimate = _estimate * pert_m;
          break;
        default:
          assert(0);
      }
    }

  };

  using VariableSim3EulerRight = VariableSim3_<VariableSim3Base::Euler, VariableSim3Base::Right>;
  using VariableSim3EulerLeft  = VariableSim3_<VariableSim3Base::Euler, VariableSim3Base::Left>;
  using VariableSim3QuaternionRight =
    VariableSim3_<VariableSim3Base::Quaternion, VariableSim3Base::Right>;
  using VariableSim3QuaternionLeft =
    VariableSim3_<VariableSim3Base::Quaternion, VariableSim3Base::Left>;

} // namespace srrg2_solver
