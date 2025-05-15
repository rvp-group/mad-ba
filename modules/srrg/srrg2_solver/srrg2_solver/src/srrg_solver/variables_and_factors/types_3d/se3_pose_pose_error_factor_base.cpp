#include "se3_pose_pose_error_factor_base.h"
#include "variable_se3.h"

namespace srrg2_solver {

  void SE3PosePoseErrorFactorBase::_drawImpl(ViewerCanvasPtr canvas_) const {
    Vector3f coords[2];
    coords[0] =
      static_cast<const VariableSE3QuaternionRight*>(variable(0))->estimate().translation();
    coords[1] =
      static_cast<const VariableSE3QuaternionRight*>(variable(1))->estimate().translation();
    canvas_->pushColor();
    float fading=1.-0.5*level();
    Vector3f color=srrg2_core::ColorPalette::color3fBlue()*fading;
    canvas_->setColor(color);
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
  }

} // namespace srrg2_solver
