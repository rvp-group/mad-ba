#include <srrg_geometry/geometry3d.h>
#include <iostream>
#include "trajectory_generator.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  // ========================== Sphere ========================== 

  void TrajectoryGeneratorSphere::compute()  {
    auto azimuth_steps=param_azimuth_steps.value();
    auto elevation_steps=param_elevation_steps.value();
    auto radius=param_radius.value();
    std::cerr << "sphere args:" << std::endl
              << "  radius: " << radius << std::endl
              << "  azimuth steps: " << azimuth_steps << std::endl
              << "  elevation steps: " << elevation_steps << std::endl;

    int steps=azimuth_steps*elevation_steps;
    float azimuth_motion=2*M_PI/azimuth_steps;
    float elevation_motion=(M_PI-0.2)/steps;
    float azimuth=0;
    float elevation=M_PI/2-0.1;
    Matrix3f Rz=geometry3d::rotationZ<float>(M_PI/2)*geometry3d::rotationX<float>(M_PI/2);
    Vector3f r(radius, 0, 0);
    for (int i=0; i<steps; ++i) {
      Eigen::Matrix3f R=geometry3d::rotationZ<float>(azimuth)*geometry3d::rotationY<float>(elevation);
      Isometry3f iso=Isometry3f::Identity();
      iso.linear()=R*Rz;
      iso.translation()=R*r;
      poses.push_back(iso);
      azimuth+=azimuth_motion;
      elevation-=elevation_motion;
    }
  }


  // ========================== Uniform UniformSphere ================== 


  void TrajectoryGeneratorSphereUniform::compute()  {
    auto steps=param_steps.value();
    auto radius=param_radius.value();

    std::cerr << "unbiform sphere args:" << std::endl
              << "  radius: " << radius << std::endl
              << "  steps: " << steps << std::endl;


    const float el_start=0.3;
    float elevation_motion=(2*M_PI)/(steps*steps);
    float azimuth=0;
    float elevation=M_PI/2-el_start;
    Matrix3f Rz=geometry3d::rotationZ<float>(M_PI/2)*geometry3d::rotationX<float>(M_PI/2);
    Vector3f r(radius, 0, 0);
    while(elevation>-M_PI/2+el_start) {
      float azimuth_motion=(2*M_PI)/(cos(elevation)*steps);
      Eigen::Matrix3f R=geometry3d::rotationZ<float>(azimuth)*geometry3d::rotationY<float>(elevation);
      Isometry3f iso=Isometry3f::Identity();
      iso.linear()=R*Rz;
      iso.translation()=R*r;
      poses.push_back(iso);
      azimuth+=azimuth_motion;
      elevation-=elevation_motion;
    }
  }

  // ========================== TORUS ========================== 

  void TrajectoryGeneratorTorus::compute() {
    auto outer_radius = param_outer_radius.value();
    auto outer_steps  = param_outer_steps.value();
    auto inner_radius = param_inner_radius.value();
    auto inner_steps  = param_inner_steps.value();
    auto laps         = param_laps.value();

    std::cerr << "torus args:" << std::endl
              << "  outer_radius: " << outer_radius << std::endl
              << "  outer steps: " <<  outer_steps << std::endl
              << "  inner_radius: " << inner_radius << std::endl
              << "  inner_steps: " <<  inner_steps << std::endl
              << "  laps: " <<  laps << std::endl;

    int steps=outer_steps*inner_steps;
    int total_steps=steps*laps;
    float outer_motion=2*M_PI/steps;
    float inner_motion=2*M_PI/inner_steps;
    float outer_angle=0;
    float inner_angle=0;
    Matrix3f Ryf=geometry3d::rotationY<float>(M_PI/2);
    Vector3f outer_r(outer_radius, 0, 0);
    Vector3f inner_r(inner_radius, 0, 0);
    for (int i=0; i<total_steps; ++i) {
      Isometry3f iso_outer=Isometry3f::Identity();
      iso_outer.linear()=geometry3d::rotationZ<float>(outer_angle);
      iso_outer.translation()=iso_outer.linear()*outer_r;

      Isometry3f iso_inner=Isometry3f::Identity();
      iso_inner.linear()=geometry3d::rotationY<float>(inner_angle);
      iso_inner.translation()=iso_inner.linear()*inner_r;
      Isometry3f iso = iso_outer*iso_inner;
      iso.linear()=iso.linear()*Ryf;
      poses.push_back(iso);
      outer_angle+=outer_motion;
      inner_angle+=inner_motion;
    }
  }

  // ========================== Manhattan ========================== 
    
  // true if agent can move forward, if true, it performs a motion
  bool TrajectoryGeneratorRandomManhattan::canMove() {
    auto grid_size = param_grid_size.value();
    auto step_size = param_step_size.value();

    ////cerr << "CanMove" << endl;
    switch(heading) {
    case 0:
      if (r>(grid_size-step_size)) {
        ////cerr << "bump right" << endl;
        return false;
      } 
      ++r;
     return true;
    case 1:
      if (c>(grid_size-step_size)){
        //cerr << "bump top" << endl;
        return false;
      }
      ++c;
      return true;
    case 2:
      if (r<(-grid_size+step_size)) {
        //cerr << "bump left" << endl;
        return false;
      }
      --r;
      return true;
    case 3:
      if (c<(-grid_size+step_size)) {
        //cerr << "bump bottom" << endl;
        return false;
      }
      --c;
      return true;
    default: return false;
    }
  }

  void TrajectoryGeneratorRandomManhattan::move() {
    auto prob_turn = param_prob_turn.value();
    auto step_size = param_step_size.value();
    double d=drand48();
    if (d>prob_turn) {
      bool move_result=false;
      for (int s=0; s<step_size; ++s) {
        move_result=canMove();
        //std::cerr << "mr: " << move_result << endl;
        if (move_result)
          addPose();
      }
      if (!move_result) { // bump makes us turning 180 degrees
        heading+=2;
        heading=heading%4;
        addPose();
      }
      return;
    }
    d=drand48();
    if (d>0.5) 
      ++heading;
    else
      --heading;
    heading=(heading+4)%4;
    addPose();
  }
    

  void TrajectoryGeneratorRandomManhattan::compute() {

    auto steps=param_steps.value();
    auto grid_size = param_grid_size.value();
    auto prob_turn = param_prob_turn.value();
    auto step_size = param_step_size.value();

    
    std::cerr << "manhattan args:" << std::endl
              << "  steps: " << steps << std::endl
              << "  grid_size: " <<  grid_size << std::endl
              << "  prob_turn: " << prob_turn << std::endl
              << "  step_size: " << step_size << std::endl;
    r=0;
    c=0;
    heading=0;
    for (int s=0; s<steps; ++s) {
      move();
    }
  }
  
  void TrajectoryGeneratorRandomManhattan::addPose() {
    Isometry3f iso;
    //cerr << r << " " << c << " " << heading << endl;
    iso.setIdentity();
    iso.translation().x()=r;
    iso.translation().y()=c;
    iso.linear()=AngleAxisf(heading*M_PI/2, Vector3f::UnitZ()).toRotationMatrix();
    poses.push_back(iso);
  }
}
