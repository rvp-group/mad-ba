#include "se3_pose_point_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE3PosePointErrorFactor::errorAndJacobian(bool error_only_) {
    const Isometry3f& X      = _variables.at<0>()->estimate();
    const Vector3f& p        = _variables.at<1>()->estimate();
    Vector3f predicted_point = X.inverse() * p;
    _e                       = predicted_point - _measurement;
    if (error_only_) {
      return;
    }
    _J.setZero();
    // tg jacobian with respect to pose
    _J.block<3, 3>(0, 0) = -1.f * Matrix3f::Identity();
    // srrg this is multiplied by 2.f because we use quaternions
    _J.block<3, 3>(0, 3) = 2.f * geometry3d::skew(predicted_point);
    // tg jacobian with respect to point
    _J.block<3, 3>(0, 6) = X.linear().transpose();
  }

  void SE3PosePointErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePointErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    const VariableSE3QuaternionRight* v_from = reinterpret_cast<const VariableSE3QuaternionRight*>(variable(0));
    const VariablePoint3* v_to   = reinterpret_cast<const VariablePoint3*>(variable(1));

    const Isometry3f& X      = v_from->estimate();
    const Vector3f& p        = v_to->estimate();
    
    coords[0] = X.translation();
    coords[1] = p;
    canvas_->pushLineWidth();
    Vector4f color;
    color.head<3>()=srrg2_core::ColorPalette::color3fGreen();
    color(3)=0.3;
    canvas_->setColor(color);
    canvas_->setLineWidth(0.1);
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
    canvas_->popAttribute();
  }

  INSTANTIATE(SE3PosePointErrorFactor)
} // namespace srrg2_solver
