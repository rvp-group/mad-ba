#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/robustifier.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_epipolar_ba_left_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_epipolar_ba_right_factor.h"
#include <srrg_geometry/geometry3d.h>
#include <iostream>
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>
#include "srrg_solver/variables_and_factors/types_sfm/se3_pairwise_scale_error_factor_ad.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

#if(__SIDE__ == 0 && __TYPE__==0)
using PoseVariableType=VariableSE3EulerLeft;
using PoseVariableTypeAD=VariableSE3EulerLeftAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorEulerLeftAD;
using EpipolarFactorType=SE3EpipolarBAEulerLeftFactorCorrespondenceDriven;
#define _FACTOR_ "SE3EpipolarBAEulerLeftFactorCorrespondenceDriven"
#endif

#if (__SIDE__==1 && __TYPE__==0)
using PoseVariableType=VariableSE3EulerRight;
using PoseVariableTypeAD=VariableSE3EulerRightAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorEulerRightAD;
using EpipolarFactorType=SE3EpipolarBAEulerRightFactorCorrespondenceDriven;
#define _FACTOR_ "SE3EpipolarBAEulerRightFactorCorrespondenceDriven"
#endif

#if(__SIDE__ == 0 && __TYPE__==1)
using PoseVariableType=VariableSE3QuaternionLeft;
using PoseVariableTypeAD=VariableSE3QuaternionLeftAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorQuaternionLeftAD;
using EpipolarFactorType=SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven;
#define _FACTOR_ "SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven"
#endif

#if (__SIDE__==1 && __TYPE__==1)
using PoseVariableType=VariableSE3QuaternionRight;
using PoseVariableTypeAD=VariableSE3QuaternionRightAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorQuaternionRightAD;
using EpipolarFactorType=SE3EpipolarBAQuaternionRightFactorCorrespondenceDriven;
#define _FACTOR_ "SE3EpipolarBAQuaternionRightFactorCorrespondenceDriven"
#endif


using ScaleFactorTypePtr=std::shared_ptr<ScaleFactorType>;

Vector6f motionVector() {
  Vector6f v;
  v << 0.1,0,0.02, 0.01,0.0,0.1;
  return v;
}
Point3fVectorCloud points_in_world;
Vector3f world_scale(5, 5, 30);
int num_points=800000;
int num_cameras = 500;
int min_matches_for_ba=8;
int max_range=1;
int min_range=0.3;
int num_fixed_cameras=1;
Vector6f motion_vector= motionVector();
float damping=0;
float measurement_noise=0.01;


std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > gt_poses, init_poses;
using PoseVariableTypePtr=std::shared_ptr<PoseVariableTypeAD>;
std::vector<PoseVariableTypePtr> camera_poses;


// stuff seen by one camera: landmark id and observation direction
struct ObservationPack{
  std::vector<int> point_indices; // landmark ids
  Point3fVectorCloud directions;  // directions
  void add(int point_idx, const Vector3f& direction) {
    point_indices.push_back(point_idx);
    Point3f p_new;
    p_new.status=POINT_STATUS::Valid;
    p_new.coordinates()=direction;
    directions.push_back(p_new);
  }
};

// all observations made by camera order matches the indices in graph
std::vector<ObservationPack> observations;

// where we assemble our stuff
FactorGraph graph;

int main(int argc, char** argv) {
  // populate the world woth points
  points_in_world.resize(num_points);
  for (auto& p: points_in_world) {
    p.coordinates()=Eigen::Vector3f::Random().array()*world_scale.array();
  }

  // spawn a bunch of observers.
  // each observer will se a piece of the world around it
  // and it will fill observations[i] with pairs "landmark id, direction"
  camera_poses.resize(num_cameras);
  observations.resize(num_cameras);
  int pose_id=0;
  Isometry3f motion=geometry3d::ta2t(motion_vector);
  Isometry3f pose=Isometry3f::Identity();
    
  for (auto& c: camera_poses) {
    c.reset(new PoseVariableTypeAD);
    c->setEstimate(pose);
    pose=pose*motion;
    c->setGraphId(pose_id);
    if (pose_id<num_fixed_cameras)
      c->setStatus(VariableBase::Fixed);
    graph.addVariable(c);
    gt_poses.push_back(c->estimate());
    auto invX=c->estimate().inverse();
    auto& op=observations[pose_id];
    for (size_t point_idx=0; point_idx<points_in_world.size(); ++point_idx) {
      Vector3f dir=(invX*points_in_world[point_idx].coordinates());
      float n=dir.norm();
      if (n>max_range || n<min_range)
        continue;
      dir += Vector3f::Random()*measurement_noise/n;
      n=dir.norm();
      op.add(point_idx, dir/n);
    }
    ++pose_id;
  };
      
  // compute the distance between the first and last pose and impose the scale
  PoseVariableTypeAD* v0=dynamic_cast<PoseVariableTypeAD*>(graph.variable(0));
  PoseVariableTypeAD* v1=dynamic_cast<PoseVariableTypeAD*>(graph.variable(num_cameras-1));
  Isometry3f x01=v0->estimate().inverse()*v1->estimate();
  float range=x01.translation().norm();

  ScaleFactorTypePtr range_factor(new ScaleFactorType);
  Eigen::Matrix<float, 1, 1> info;
  info(0,0)=1e3;
  range_factor->setInformationMatrix(info);
  range_factor->setRange(range);
  range_factor->setVariableId(0,0);
  range_factor->setVariableId(1,num_cameras-1);
  graph.addFactor(range_factor);
  
  // we scan each pair of variables, and we count if they observe enough common landmarks
  // since the landmarks are ordered, we can do a linear search
  // for a pair of cameras [i,j] we fill the correspondence vector
  // a correspondence is a pair of indices. They represent the position
  // in the observation vector of camera i and the position in observation vector of camera j
  // that point to rhe same entity.
  // if enough common landmarks are found, we create a correspondence vector and store it in the list
  // we also create  an epipolar factor between the two variables and we load it with the  (stored)
  // correspondences.
  
  int total_measurements=0;
  for (size_t i=0; i<camera_poses.size()-1; ++i) {
    const auto& o1=observations[i];
    int num_factors_for_pose=0;
    for (size_t j=i+1; j<camera_poses.size(); ++j) {
      const auto& o2=observations[j];
      size_t ii=0;
      size_t jj=0;
      CorrespondenceVector correspondences;
      correspondences.resize(std::min(o1.point_indices.size(), o2.point_indices.size()));
      int k=0;
      while (ii<o1.point_indices.size() && jj<o2.point_indices.size()) {
        int i1=o1.point_indices[ii];
        int i2=o2.point_indices[jj];
        if (i1==i2) {
          correspondences.at(k).fixed_idx=ii;
          correspondences.at(k).moving_idx=jj;
          ++ii;
          ++jj;
          ++k;
          continue;
        }
        if (i1<i2) {
          ++ii;
          continue;
        }
        if (i2<i1) {
          ++jj;
          continue;
        }
      }
      correspondences.resize(k);
      if (k>min_matches_for_ba) {
        EpipolarFactorType* factor=new EpipolarFactorType;

        factor->setVariableId(0,i);
        factor->setVariableId(1,j);
        factor->storeCorrespondences(correspondences);
        factor->setFixed(o1.directions);
        factor->setMoving(o2.directions);
        graph.addFactor(std::shared_ptr<EpipolarFactorType>(factor));
        total_measurements+=k;
        ++ num_factors_for_pose;
      } else {
        //cerr << "poses [" << i << "," << j << "]-> " << k << "NOPE" << endl;
      }
    }
    cerr << "pose: " << i << " # factors: " << num_factors_for_pose << endl;
  }
  cerr << "num_variables: " << graph.variables().size() << endl;
  cerr << "num_factors:   " << graph.factors().size() << endl;
  cerr << "total_pairwise_point_measurements:   " << total_measurements << endl;
  cerr << "fill in: " << 2*(float)graph.factors().size()/(float)(graph.variables().size()*(graph.variables().size()+1)) << endl;


  // instantiate and configure the solver
  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> alg(new IterationAlgorithmGN);
  RobustifierPolicyByTypePtr policy(new RobustifierPolicyByType);
  policy->param_factor_class_name.setValue(_FACTOR_);
  std::shared_ptr<RobustifierCauchy> robustifier(new RobustifierCauchy);
  policy->param_robustifier.setValue(robustifier);
  robustifier->param_chi_threshold.setValue(1e-3);
  //solver.param_robustifier_policies.pushBack(policy);
  alg->param_damping.setValue(1);
  std::shared_ptr<SparseBlockLinearSolverCholeskyCSparse> iter(new SparseBlockLinearSolverCholeskyCSparse);
  solver.param_verbose.setValue(true);
  solver.param_algorithm.setValue(alg);
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_linear_solver.setValue(iter);
  solver.param_max_iterations.setValue({100});

  // now we have fun and simulate a growing noise along our chain of poses
  cerr << "now messing up poses" << endl;
  Vector6f noise_stats;
  noise_stats << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
  noise_stats += 0.5*motion_vector;
  Isometry3f noise=Isometry3f::Identity();
  int k=0;
  Isometry3f previous_camera=Isometry3f::Identity();
  Isometry3f previous_camera_noise=Isometry3f::Identity();
  for(auto& c: camera_poses) {
    Isometry3f new_camera=c->estimate();
    Isometry3f dc = previous_camera.inverse()*new_camera;
    previous_camera_noise = previous_camera_noise*dc*noise;
    c->setEstimate(previous_camera_noise);
    previous_camera=new_camera;
    init_poses.push_back(c->estimate());
    ++k;
    if (k<num_fixed_cameras){
      continue;
    }
    Vector6f sample=  noise_stats.array()  * Vector6f::Random().array();
    noise=geometry3d::ta2t(sample);
  }

  // solve and see what happens
  solver.setGraph(graph);
  SolverActionBasePtr act(new SolverVerboseAction);
  solver.param_actions.pushBack(act);
  solver.compute();
  cerr << solver.iterationStats() << endl;
  ofstream os ("ba_out.txt");
  //os << "#gtx gty gtz gtqx gtqy gtqz initx inity initz initqx initqy initqz estx esty estz estqx estqy estqz" << endl;
  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f init=init_poses[i];
    Isometry3f gt=gt_poses[i];
    cerr << "pose: " << i;
    PoseVariableTypeAD* v=dynamic_cast<PoseVariableTypeAD*>(graph.variable(i));
    AngleAxisf aa(gt.linear().transpose()*v->estimate().linear());
    AngleAxisf aai(gt.linear().transpose()*init.linear());
    cerr << "  rot_err: " << aai.angle() << " -> " << aa.angle();
    cerr << " tras_err: (" << (init.translation()-gt.translation()).norm() <<") -> (" << (v->estimate().translation()-gt.translation()).norm() << ")" << endl;
    os << geometry3d::t2v(gt).transpose()
       << " " << geometry3d::t2v(init).transpose()
       << " " << geometry3d::t2v(v->estimate()).transpose() << endl;;
 }
}
