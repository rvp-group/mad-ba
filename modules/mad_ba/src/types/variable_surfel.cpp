#include "variable_surfel.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  void VariableSurfel::setZero()  {
    setEstimate(Isometry3f::Identity());
  }

  void VariableSurfel::applyPerturbation(const Vector5f& pert) {
      this->_updated = true;
      Vector6f pert_full;
      pert_full.block<5,1>(0,0)=pert;
      pert_full(5)=0;
      _estimate= _estimate * geometry3d::v2t(pert_full);
  }

    void VariableSurfel::_drawImpl(ViewerCanvasPtr canvas_) const  {
    if (!canvas_)
      throw std::runtime_error("VariableSE3_::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->pushMatrix();
    canvas_->multMatrix(_estimate.matrix());
    //      canvas_->putSphere(0.1);
    canvas_->putReferenceSystem(3);
    canvas_->popMatrix();
    canvas_->popAttribute();
  }

    void VariableSurfel1D::setZero()  {
    setEstimate(Isometry3f::Identity());
  }

  void VariableSurfel1D::applyPerturbation(const Vector1f& pert) {
      this->_updated = true;
      float dZ = pert(0,0);
      _estimate.translation() += _estimate.linear().col(2) * dZ;
  }

    void VariableSurfel1D::_drawImpl(ViewerCanvasPtr canvas_) const  {
    if (!canvas_)
      throw std::runtime_error("VariableSE3_::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlue());
    canvas_->pushMatrix();
    canvas_->multMatrix(_estimate.matrix());
    //      canvas_->putSphere(0.1);
    canvas_->putReferenceSystem(3);
    canvas_->popMatrix();
    canvas_->popAttribute();
  }

}
