#include "se3_pose_surfel_factor_ad.h"
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver {

    void SE3PoseSurfelQuaternionErrorFactorAD::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePoseChordalEulerErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    coords[0] = static_cast<const VariableSE3QuaternionRightAD*>(variable(0))->estimate().translation();
    coords[1] = static_cast<const VariableSurfel*>(variable(1))->estimate().translation();
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
  }

  INSTANTIATE(SE3PoseSurfelQuaternionErrorFactorAD)

    void SE3PoseSurfelQuaternionErrorFactorAD1D::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePoseChordalEulerErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    coords[0] = static_cast<const VariableSE3QuaternionRightAD*>(variable(0))->estimate().translation();
    coords[1] = static_cast<const VariableSurfel1D*>(variable(1))->estimate().translation();
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
  }

  INSTANTIATE(SE3PoseSurfelQuaternionErrorFactorAD1D)
}
