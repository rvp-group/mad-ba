#include <srrg_geometry/geometry2d.h>
#include <srrg_geometry/geometry3d.h>

#include <srrg_geometry/geometry_defs.h>

#include <iostream>


using namespace srrg2_core;
using namespace srrg2_core::geometry2d;
using namespace srrg2_core::geometry3d;

#warning "Move me to TEST folder"

int main (int argc, char ** argv) {

  /* Test on Geometry 2d
   * 
   * */    
  std::cerr << "\n****** Test on 2d Geometry ************ \n";
  std::cerr << "[TEST]: v2t - t2v \n\n";  
  Vector3f pose;
  pose << 0.3, 0.5, 1.57;
  std::cerr << "pose:           " << pose.transpose() << std::endl;
  Isometry2f iso = v2t(pose);
  std::cerr << "iso:\n" << iso.matrix() << std::endl;
  Vector3f b_pose = t2v(iso);
  std::cerr << "retrieved pose: " << b_pose.transpose() << std::endl; 

  std::cerr << "\n[TEST]: euclidean2polar and viceversa \n\n";  
  Vector2f euclidean, polar;
  euclidean << 0.5, -0.5;
  std::cerr << "euclidean:      " << euclidean.transpose() << std::endl;
  polar = euclidean2polar(euclidean);
  std::cerr << "polar:          " << polar.transpose() << std::endl;    
  std::cerr << "retrieved eucl: " << polar2euclidean(polar).transpose() << std::endl;
  /* Test on Geometry 3d
   * 
   * */    
  std::cerr << "\n****** Test on 3d Geometry ************ \n";
  std::cerr << "[TEST]: v2t - t2v \n\n";
  Vector6f pose3;
  pose3 << 0.3, 0.5, 0.2, 0.5, -0.5, 0.5;
  std::cerr << "pose 3d:         " << pose3.transpose() << std::endl;   
  Isometry3f iso3 = v2t(pose3);
  std::cerr << "iso:\n" << iso3.matrix() << std::endl;
  Vector6f b_pose3 = t2v(iso3);
  std::cerr << "retrieved pose:  " << b_pose3.transpose() << std::endl;     
  
  std::cerr << "\n[TEST]: ta2t - t2ta \n\n";    
  Vector6f ta;
  ta << 0.1, -0.9, 1.1, 1.57, -1.1, 3.01;
  std::cerr << "transAng:        " << ta.transpose() << std::endl;
  Isometry3f taiso = ta2t(ta);
  std::cerr << "iso:\n" << taiso.matrix() << std::endl;
  std::cerr << "retrieved pose:  " << t2ta(taiso).transpose() << std::endl;     
  
  std::cerr << "\n[TEST]: euclidean2polar and viceversa \n\n";
  Vector3f euclidean3, polar3;
  euclidean3 << 0.5, 0.6, 10;
  std::cerr << "euclidean:       " << euclidean3.transpose() << std::endl;
  polar3 = euclidean2polar(euclidean3);
  std::cerr << "polar:           " << polar3.transpose() << std::endl;
  std::cerr << "retrieved eucl:  " << polar2euclidean(polar3).transpose() << std::endl;
  
  std::cerr << "\n[TEST]: hom vector \n\n";
  Vector3f unhomogeneous_v;
  unhomogeneous_v << 0.5, 1.3, 0.8;
  std::cerr << "unhomogeneous:   " << unhomogeneous_v.transpose() << std::endl;
  std::cerr << "homogeneous v.:  " << hom(unhomogeneous_v).transpose() << std::endl;

  std::cerr << "\n[TEST]: 3d->2d extraction \n\n";
  pose3 << 0.5, 0.2, 1.2, 0.0, 0.0, 0.523599;
  std::cerr << "pose 3d:        " << pose3.transpose() << std::endl;
  Isometry2f pose_2d = get2dFrom3dPose(ta2t(pose3));
  std::cerr << "pose 2d:        " << t2v(pose_2d).transpose() << std::endl;
    
  return 0;
}

