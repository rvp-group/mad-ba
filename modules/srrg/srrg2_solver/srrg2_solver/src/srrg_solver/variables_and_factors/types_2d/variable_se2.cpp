#include "variable_se2.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  using namespace srrg2_core;

  void VariableSE2Base::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("VariableSE2_::draw|invalid canvas");
    }
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fDarkRed());
    canvas_->pushMatrix();
    canvas_->multMatrix(geometry3d::get3dFrom2dPose(_estimate).matrix());
    canvas_->putSphere(0.1);
    //    canvas_->putReferenceSystem(0.1);
    canvas_->popMatrix();
    canvas_->popAttribute();
  }

  INSTANTIATE(VariableSE2Right)
  INSTANTIATE(VariableSE2Left)

} // namespace srrg2_solver
