#include "se3_pose_point_rectified_stereo_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  
  inline bool inside(const Vector3f& image_point, const Vector3f& sizes) {
    return true;
    return image_point.x() >= 0 && image_point.y() >= 0
      && image_point.x()<=sizes.x() && image_point.y()<=sizes.y()
      && image_point.z() > 0
      && (image_point.x()-image_point.z()) >=0;
    
  }
  
  bool SE3PosePointRectifiedStereoErrorFactor::triangulate(Vector3f& point_in_world,
                                                           const Vector3f& m,
                                                           const VariableSE3QuaternionRight& pose,
                                                           const VariableMatrix3_4& proj,
                                                           const VariablePoint3& sizes_and_baseline) {
    const auto& projection_matrix=proj.estimate();
    const auto& image_size=sizes_and_baseline.estimate();
    const auto& baseline=image_size.z();
    if (! inside(m, image_size))
      return false;
    float z=baseline/m.z();
    Eigen::Vector3f p_cam(m.x()*z, m.y()*z, z);
    Eigen::Vector3f p_local=projection_matrix.block<3,3>(0,0).inverse()
      *(p_cam-projection_matrix.block<3,1>(0,3));
    point_in_world=pose.estimate()*p_local;
    return true;
  }


  bool SE3PosePointRectifiedStereoErrorFactor::computePrediction(Vector3f& prediction,
                                                           const VariableSE3QuaternionRight& pose,
                                                           const VariablePoint3& point,
                                                           const VariableMatrix3_4& proj,
                                                           const VariablePoint3& sizes) {

    const auto& robot_pose=pose.estimate();
    const auto& point_in_world=point.estimate();
    const auto& projection_matrix=proj.estimate();
    const auto& image_size=sizes.estimate();
    const auto& baseline=image_size.z();
    
    auto point_in_robot = robot_pose.inverse() * point_in_world;
    Eigen::Matrix<float,4,4> extended_projection_matrix;
    extended_projection_matrix.block<3,4>(0,0)=projection_matrix;
    extended_projection_matrix.block<1,4>(3,0)=projection_matrix.block<1,4>(0,0);
    extended_projection_matrix(3,3) -= baseline;

    auto point_in_cam   = extended_projection_matrix.block<4,3>(0,0)*point_in_robot
      + extended_projection_matrix.block<4,1>(0,3);

    const float xl=point_in_cam(0);
    const float xr=point_in_cam(3);
    const float y=point_in_cam(1);
    const float z=point_in_cam(2);
    
    if (fabs(z)<0)
      return false;
    const float iz=1./z;
    prediction <<
      xl *iz,
      y *iz,
      (xl-xr) *iz;
    
    return  inside(prediction,image_size);
  }

  void SE3PosePointRectifiedStereoErrorFactor::errorAndJacobian(bool error_only_) {
    _is_valid = false;
    
    // retrieve the  parameters
    auto v_robot_pose        = _variables.at<0>();
    auto v_point_in_world    = _variables.at<1>();
    auto v_projection_matrix = _variables.at<2>();
    auto v_image_size        = _variables.at<3>();

    if ( v_projection_matrix->status()!=VariableBase::Fixed )
      throw std::runtime_error("SE3PosePointRectifiedStereoErrorFactor: projection_matrix not fixed");
    if ( v_image_size->status()!=VariableBase::Fixed)
      throw std::runtime_error("SE3PosePointRectifiedStereoErrorFactor: image_size not fixed");

    const auto& robot_pose=v_robot_pose->estimate();
    const auto& point_in_world=v_point_in_world->estimate();
    const auto& projection_matrix=v_projection_matrix->estimate();
    const auto& image_size=v_image_size->estimate();
    const auto& baseline=image_size.z();

    // assemble an extended proj matrix
    // that projects x_l, y_l, z_l, x_r
    Eigen::Matrix<float,4,4> extended_projection_matrix;
    extended_projection_matrix.block<3,4>(0,0)=projection_matrix;
    extended_projection_matrix.block<1,4>(3,0)=projection_matrix.block<1,4>(0,0);
    extended_projection_matrix(3,3) -= baseline;
    
    auto point_in_robot = robot_pose.inverse() * point_in_world;
    auto point_in_cam   = extended_projection_matrix.block<4,3>(0,0)*point_in_robot
      + extended_projection_matrix.block<4,1>(0,3);


    const float xl=point_in_cam(0);
    const float xr=point_in_cam(3);
    const float y=point_in_cam(1);
    const float z=point_in_cam(2);
    
    if (fabs(z)<z_cutoff)
      return;
    const float iz=1./z;
    Vector3f prediction (xl *iz,
                         y *iz,
                         (xl-xr) *iz);
    if (0 && ! inside(prediction,image_size) ) {
      return;
    }
    _is_valid=true;
    _e = prediction - _measurement;
    if (error_only_)
      return;

    Matrix3_4f J_hom;
    const float iz2=iz*iz;
    J_hom <<
      iz,   0, -xl*iz2,   0,
      0 ,  iz, -y*iz2, 0,
     iz,  0,  (xr-xl)* iz2, -iz;

    // robot_pose
    Matrix4_6f J_cam;
    J_cam.block<4,3>(0,0) = -extended_projection_matrix.block<4,3>(0,0);
    J_cam.block<4,3>(0,3) = 2 * extended_projection_matrix.block<4,3>(0,0) * geometry3d::skew(point_in_robot);
    jacobian<0>() = J_hom*J_cam;

    // point
    jacobian<1>() =  J_hom * extended_projection_matrix.block<4,3>(0,0) * robot_pose.linear().transpose();
  }

  void SE3PosePointRectifiedStereoErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const {
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
  
  INSTANTIATE(SE3PosePointRectifiedStereoErrorFactor)

} // namespace srrg2_solver
