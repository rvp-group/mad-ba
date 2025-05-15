#include "se3_pose_point_pinhole_ba_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  bool SE3PosePointPinholeBAErrorFactor::computePrediction(Vector2f& prediction,
                                                           const VariableSE3QuaternionRight& pose,
                                                           const VariablePoint3& point,
                                                           const VariableMatrix3_4& proj,
                                                           const VariablePoint2& sizes) {

    const auto& robot_pose=pose.estimate();
    const auto& point_in_world=point.estimate();
    const auto& projection_matrix=proj.estimate();
    const auto& image_size=sizes.estimate();
    
    auto point_in_robot = robot_pose.inverse() * point_in_world;
    auto point_in_cam   = projection_matrix.block<3,3>(0,0)*point_in_robot
      + projection_matrix.block<3,1>(0,3);
    if (point_in_cam.z()<0)
      return false;
    prediction = Vector2f (point_in_cam.x()/point_in_cam.z(),
                           point_in_cam.y()/point_in_cam.z());
    if (prediction.x() < 0
        || prediction.y() <0
        || prediction.x() > image_size.x()
              || prediction.y() > image_size.y())
      return false;

    return true;
  }

  void SE3PosePointPinholeBAErrorFactor::errorAndJacobian(bool error_only_) {
    _is_valid = false;
    
    // retrieve the  parameters
    auto v_robot_pose        = _variables.at<0>();
    auto v_point_in_world    = _variables.at<1>();
    auto v_projection_matrix = _variables.at<2>();
    auto v_image_size        = _variables.at<3>();

    if ( v_projection_matrix->status()!=VariableBase::Fixed )
      throw std::runtime_error("SE3PosePointPinholeBAErrorFactor: projection_matrix not fixed");
    if ( v_image_size->status()!=VariableBase::Fixed)
      throw std::runtime_error("SE3PosePointPinholeBAErrorFactor: image_size not fixed");

    const auto& robot_pose=v_robot_pose->estimate();
    const auto& point_in_world=v_point_in_world->estimate();
    const auto& projection_matrix=v_projection_matrix->estimate();
    const auto& image_size=v_image_size->estimate();
    
    auto point_in_robot = robot_pose.inverse() * point_in_world;
    auto point_in_cam   = projection_matrix.block<3,3>(0,0)*point_in_robot
      + projection_matrix.block<3,1>(0,3);
    if (fabs(point_in_cam.z())<z_cutoff)
      return;
    const float iz=1./point_in_cam.z();
    Vector2f prediction (point_in_cam.x() *iz,
                         point_in_cam.y() *iz );
    if (0 && (prediction.x() < 0
              || prediction.y() <0
              || prediction.x() > image_size.x()
              || prediction.y() > image_size.y()))
      return;

    _is_valid=true;
    _e = prediction - _measurement;
    if (error_only_)
      return;

    Matrix2_3f J_hom;
    const float iz2=iz*iz;
    J_hom <<
      iz,  0, -point_in_cam.x()*iz2,
      0 , iz, -point_in_cam.y()*iz2;

    // robot_pose
    Matrix3_6f J_cam;
    J_cam.block<3,3>(0,0) = -projection_matrix.block<3,3>(0,0);
    J_cam.block<3,3>(0,3) = 2 * projection_matrix.block<3,3>(0,0) * geometry3d::skew(point_in_robot);
    jacobian<0>() = J_hom*J_cam;

    // point
    jacobian<1>() =  J_hom * projection_matrix.block<3,3>(0,0) * robot_pose.linear().transpose();
  }

  void SE3PosePointPinholeBAErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const {
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
  
  INSTANTIATE(SE3PosePointPinholeBAErrorFactor)

} // namespace srrg2_solver
