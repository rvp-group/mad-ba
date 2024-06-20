#include "variable_se3.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  void VariableSE3Base::setZero()  {
    setEstimate(Isometry3f::Identity());
  }

  void VariableSE3Base::_drawImpl(ViewerCanvasPtr canvas_) const  {
    if (!canvas_)
      throw std::runtime_error("VariableSE3_::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->pushMatrix();
    canvas_->multMatrix(_estimate.matrix());
    //      canvas_->putSphere(0.1);
    canvas_->putReferenceSystem(1);
    canvas_->popMatrix();
    canvas_->popAttribute();
  }

  void VariableSE3Base::normalize() {
    Matrix3f R=_estimate.linear();
    Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _estimate.linear() -= 0.5 * R * E;
    setEstimate(_estimate);
  }

  INSTANTIATE(VariableSE3EulerRight)
  INSTANTIATE(VariableSE3EulerLeft)
  INSTANTIATE(VariableSE3QuaternionRight)
  INSTANTIATE(VariableSE3QuaternionLeft)
  
} // namespace srrg2_solver
