#include "srrg_geometry/geometry3d.h"
#include <iostream>
#include <Eigen/StdVector>
#include "srrg_autodiff/ad_multivariate.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_core::geometry3d;

using namespace Eigen;

typedef srrg2_core::Vector2ad<float> Vector2adf;
typedef srrg2_core::Vector3ad<float> Vector3adf;
typedef srrg2_core::Vector6ad<float> Vector6adf;
typedef srrg2_core::Matrix3ad<float> Matrix3adf;
typedef srrg2_core::DualValue_<float> DualValuef;
typedef srrg2_core::Isometry3ad<float> Isometry3adf;



  
template <typename Scalar_>
class TransformPoint: public MultivariateFunction<Scalar_, 6, 3>{
public:
  void operator()(Scalar_* output, const Scalar_* input){

    // this maps the memory area in the input array to an
    // Eigen vector of dimension 6
    // if you feel uncomfy, you can
    // allocate an array
    //   Vector6<Scalar_> robot_pose;
    // fill the elements with a for loop
    //    for (int i=0; i<6; i++) robot_pose[i]=input[i];
    Eigen::Map<const Eigen::Matrix<Scalar_,6,1> > robot_pose(input);

    // this maps the memory area in the output array to an
    // Eigen vector of dimension 3
    // Changing the Eigen object would result
    // in doing side effect to the memory area
    Eigen::Map<Eigen::Matrix<Scalar_,3,1> > projected_point(output);

    
    // compute the rotation matrix and translation vector
    // encoded in robot_pose
    Isometry3_<Scalar_> robot_pose_matrix=v2t<Scalar_>(robot_pose);
   
    // compute the positionof the point w.r.t. the world,
    // by multiplying it by a transformation matrix 
    projected_point=robot_pose_matrix*point;
  }

  // this is the parameter
  Eigen::Matrix<Scalar_, 3, 1> point;
};


// this is our function that supports the multivariate autodiff
ADMultivariateFunction<double, TransformPoint> ad_project_point;


  
template <typename Scalar_>
class TransformMovingPoint: public MultivariateFunction<Scalar_, 9, 3>{
public:
  void operator()(Scalar_* output, const Scalar_* input){

    // this maps the memory area in the input array to an
    // Eigen vector of dimension 6
    // if you feel uncomfy, you can
    // allocate an array
    //   Vector6<Scalar_> robot_pose;
    // fill the elements with a for loop
    //    for (int i=0; i<6; i++) robot_pose[i]=input[i];
    Eigen::Map<const Eigen::Matrix<Scalar_,6,1> > robot_pose(input);
    Eigen::Map<const Eigen::Matrix<Scalar_,3,1> > point(input+6);

    
    // this maps the memory area in the output array to an
    // Eigen vector of dimension 3
    // Changing the Eigen object would result
    // in doing side effect to the memory area
    Eigen::Map<Eigen::Matrix<Scalar_,3,1> > projected_point(output);

    
    // compute the rotation matrix and translation vector
    // encoded in robot_pose
    Isometry3_<Scalar_> robot_pose_matrix=v2t<Scalar_>(robot_pose);
   
    // compute the positionof the point w.r.t. the world,
    // by multiplying it by a transformation matrix 
    projected_point=robot_pose_matrix*point;
  }
};

template <typename Scalar_>
class TransformMovingPoint2: public MultivariateFunction<Scalar_, 9, 3>{
public:
  void operator()(Scalar_* output, const Scalar_* input){

    // this maps the memory area in the input array to an
    // Eigen vector of dimension 6
    // if you feel uncomfy, you can
    // allocate an array
    //   Vector6<Scalar_> robot_pose;
    // fill the elements with a for loop
    //    for (int i=0; i<6; i++) robot_pose[i]=input[i];
    Eigen::Map<const Eigen::Matrix<Scalar_,6,1> > robot_pose_pert(input);
    Eigen::Map<const Eigen::Matrix<Scalar_,3,1> > point_pert(input+6);

    
    // this maps the memory area in the output array to an
    // Eigen vector of dimension 3
    // Changing the Eigen object would result
    // in doing side effect to the memory area
    Eigen::Map<Eigen::Matrix<Scalar_,3,1> > projected_point(output);

    
    // compute the rotation matrix and translation vector
    // encoded in robot_pose
    Isometry3_<Scalar_> robot_pose_matrix=v2t<Scalar_>(robot_pose_pert)*_robot_pose;
   
    // compute the positionof the point w.r.t. the world,
    // by multiplying it by a transformation matrix 
    projected_point=robot_pose_matrix*(_point+point_pert);
  }
  Isometry3_<Scalar_> _robot_pose;
  Vector3_<Scalar_> _point;
};



// this is our function that supports the multivariate autodiff
ADMultivariateFunction<double, TransformMovingPoint> ad_move_point;

ADMultivariateFunction<double, TransformMovingPoint2> ad_move_point_manifold;

int main(int argc, char** argv){
  ad_project_point.point<<1,2,3;
  ad_project_point.point<<1,2,3;

  Eigen::Matrix<double, 6, 1> v;
  v << 0,0,0,0,0,0;
  
  Eigen::Matrix<double, 3, 1> output;
  Eigen::Matrix<double, 3, 6> jacobian;
  
  ad_project_point(&output[0], &v[0]);
  jacobian=ad_project_point.jacobian(&v[0]);

  cerr << "output: " << endl;
  cerr << output.transpose() << endl;
  cerr << "jacobian: " << endl;
  cerr << jacobian << endl;

  cerr << endl  << endl;
  Eigen::Matrix<double, 9, 1> pose_and_point;
  Eigen::Matrix<double, 3, 9> jacobian_move;
  pose_and_point.head<6>()=v;
  pose_and_point.tail<3>() << 1,2,3;
  cerr << "pose_and_point: " <<pose_and_point.transpose() << endl;
  ad_move_point(&output[0], &pose_and_point[0]);
  jacobian_move=ad_move_point.jacobian(&pose_and_point[0]);
  cerr << "jacobian: " << endl;
  cerr << jacobian_move << endl;

  cerr << "pose_and_point (zero pert): " <<pose_and_point.transpose() << endl;
  ad_move_point_manifold._robot_pose.setIdentity();
  ad_move_point_manifold._point << 1,2,3;
  ad_move_point_manifold(&output[0]);
  jacobian_move=ad_move_point_manifold.jacobian();
  cerr << "jacobian: " << endl;
  cerr << jacobian_move << endl;

}
