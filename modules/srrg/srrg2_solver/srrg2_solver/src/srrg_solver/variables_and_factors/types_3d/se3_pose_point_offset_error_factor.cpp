#include "se3_pose_point_offset_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void SE3PosePointOffsetErrorFactor::errorAndJacobian(bool error_only_) {
    VariableSE3QuaternionRight* v_from   = _variables.at<0>();
    VariablePoint3* v_point              = _variables.at<1>();
    VariableSE3QuaternionRight* v_offset = _variables.at<2>();

    assert(v_from && "SE3PosePointOffsetErrorFactor::errorAndJacobian|invalid cast v_from");
    assert(v_point && "SE3PosePointOffsetErrorFactor::errorAndJacobian|invalid cast v_point");
    assert(v_offset && "SE3PosePointOffsetErrorFactor::errorAndJacobian|invalid cast v_offset");

    const Isometry3f& from   = v_from->estimate();
    const Vector3f& point    = v_point->estimate();
    const Isometry3f& offset = v_offset->estimate();

    const Isometry3f total_transform = from * offset;
    const Vector3f predicted_point   = total_transform.inverse() * point;
    _e                               = predicted_point - _measurement;
    if (error_only_) {
      return;
    }

    auto J_from                 = jacobian<0>();
    const Matrix3f& R_transpose = offset.linear().transpose();
    J_from.setZero();
    J_from.block<3, 3>(0, 0)                = -1.0f * R_transpose;
    Vector3f predicted_point_in_robot_frame = from.inverse() * point;
    J_from.block<3, 3>(0, 3) =
      2.0f * R_transpose * geometry3d::skew(predicted_point_in_robot_frame);

    auto J_point            = jacobian<1>();
    const Matrix3f& R_total = total_transform.linear();
    J_point                 = R_total.transpose();

    auto J_offset              = jacobian<2>();
    J_offset.block<3, 3>(0, 0) = -Matrix3f::Identity();
    J_offset.block<3, 3>(0, 3) = 2.0f * geometry3d::skew(predicted_point);
  }

  void SE3PosePointOffsetErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePointOffsetErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
        const VariableSE3QuaternionRight* v_from = reinterpret_cast<const VariableSE3QuaternionRight*>(variable(0));
    const VariablePoint3* v_to   = reinterpret_cast<const VariablePoint3*>(variable(1));
    const Isometry3f& X      = v_from->estimate();
    const Vector3f& p        = v_to->estimate();
    coords[0] = X.translation();
    coords[1] = p;
    canvas_->pushColor();
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
  INSTANTIATE(SE3PosePointOffsetErrorFactor)

} // namespace srrg2_solver
