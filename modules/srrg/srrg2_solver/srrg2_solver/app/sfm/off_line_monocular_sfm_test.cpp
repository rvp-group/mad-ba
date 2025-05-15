#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_nullspace.h"
#include "srrg_solver/solver_core/robustifier.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_epipolar_ba_left_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_epipolar_ba_right_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/essential_translation_factor.h"
#include <srrg_geometry/geometry3d.h>
#include <iostream>
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>
#include <srrg_solver/variables_and_factors/types_sfm/variable_so3.h>
#include "srrg_solver/variables_and_factors/types_sfm/so3_pose_pose_error_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_pairwise_scale_error_factor_ad.h"
#include "srrg_geometry/epipolar.h"
#include <Eigen/SparseQR>

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

#if(__SIDE__ == 0)
using PoseVariableType=VariableSE3QuaternionLeft;
using PoseVariableTypeAD=VariableSE3QuaternionLeftAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorQuaternionLeftAD;
using EpipolarFactorType=SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven;
#define _FACTOR_ "SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven"
#endif

#if (__SIDE__==1)
using PoseVariableType=VariableSE3QuaternionRight;
using PoseVariableTypeAD=VariableSE3QuaternionRightAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorQuaternionRightAD;
using EpipolarFactorType=SE3EpipolarBAQuaternionRightFactorCorrespondenceDriven;
#define _FACTOR_ "SE3EpipolarBAQuaternionRightFactorCorrespondenceDriven"
#endif

using ScaleFactorTypePtr=std::shared_ptr<ScaleFactorType>;

Vector6f motionVector() {
  Vector6f v;
  v << 0.1,0,0.01, 0.01,0.0,0.1;
  return v;
}
Point3fVectorCloud points_in_world;
Vector3f world_scale(5, 5, 30);
int num_points=800000;
int num_cameras = 1000;
int min_matches_for_ba=8;
int max_range=1;
int min_range=0.3;
int num_fixed_cameras=1;
Vector6f motion_vector= motionVector();
float damping=0;
float measurement_noise=0.01;
int min_matches_for_essential=30;
float ransac_inlier_threshold = 0.01;
int ransac_num_rounds = 100;
int min_ransac_inliers = 20;

std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > gt_poses, init_poses;
using PoseVariableTypePtr=std::shared_ptr<PoseVariableTypeAD>;
std::vector<PoseVariableTypePtr> camera_poses;

using VariableSO3RelaxedPtr = std::shared_ptr<VariableSO3Relaxed>;
using SO3RelaxedPosePoseErrorFactorPtr = std::shared_ptr<SO3RelaxedPosePoseErrorFactor>;

using VariableSO3Ptr = std::shared_ptr<VariableSO3QuaternionRightAD>;
using SO3PosePoseQuaternionRightErrorFactorADPtr = std::shared_ptr<SO3PosePoseQuaternionRightErrorFactorAD>;

using VariableTranslation = VariablePoint3AD;
using VariableTranslationPtr = std::shared_ptr<VariableTranslation>;
using TranslationFactor = EssentialTranslationFactor;
using TranslationFactorPtr = std::shared_ptr<TranslationFactor>;

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
FactorGraph ba_graph;
FactorGraph orientation_graph;
FactorGraph orientation_nonlinear_graph;
FactorGraph translation_graph;

int main(int argc, char** argv) {
  // populate the world woth points
  points_in_world.resize(num_points);
  for (auto& p: points_in_world) {
    p.coordinates()=Eigen::Vector3f::Random().array()*world_scale.array();
  }

  /*********** GENERATE DATA AND POPULATE VARIABLES ***********/ 
  // spawn a bunch of observers.
  // each observer will se a piece of the world around it
  // and it will fill observations[i] with pairs "landmark id, direction"
  camera_poses.resize(num_cameras);
  observations.resize(num_cameras);
  int pose_id=0;
  Isometry3f motion=geometry3d::ta2t(motion_vector);
  Isometry3f pose=Isometry3f::Identity();
  double squared_translation_norm=0;
  for (auto& c: camera_poses) {
    c.reset(new PoseVariableTypeAD);
    c->setEstimate(pose);
    pose=pose*motion;
    c->setGraphId(pose_id);
    squared_translation_norm+=c->estimate().translation().squaredNorm();
    
    VariableSO3RelaxedPtr orientation_var(new VariableSO3Relaxed);
    orientation_var->setGraphId(pose_id);
    //orientation_var->setEstimate(c->estimate().linear());
    orientation_var->setZero();

    VariableSO3Ptr orientation_nl_var(new VariableSO3QuaternionRightAD);
    orientation_nl_var->setGraphId(pose_id);
    orientation_nl_var->setZero();

    VariableTranslationPtr translation_var(new VariableTranslation);
    translation_var->setGraphId(pose_id);
    translation_var->setZero();
    
    if (pose_id<num_fixed_cameras) {
      c->setStatus(VariableBase::Fixed);
      orientation_var->setEstimate(c->estimate().linear());
      orientation_nl_var->setEstimate(c->estimate().linear());
      translation_var->setEstimate(c->estimate().translation());
      
      orientation_var->setStatus(VariableBase::Fixed);
      orientation_nl_var->setStatus(VariableBase::Fixed);
      translation_var->setStatus(VariableBase::Fixed);
    } else {
      translation_var->setEstimate(Vector3f::Zero());
    }
    
    ba_graph.addVariable(c);
    orientation_graph.addVariable(orientation_var);
    orientation_nonlinear_graph.addVariable(orientation_nl_var);
    translation_graph.addVariable(translation_var);
    
    gt_poses.push_back(c->estimate());

    
  /*********** GENERATE OBSERVATIONS FOR EACH FRAME ***********/ 

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

  /*********** ADD ABSOLUTE SCALE CONSTRAINTS TO BA AND TRANSLATION PROBLEM ***********/ 

  // compute the magnufication of the nullspace for the translation problem
  float translation_scale=sqrt(squared_translation_norm);
  
  // compute the distance between the first and last pose and impose the scale
  PoseVariableTypeAD* v0=dynamic_cast<PoseVariableTypeAD*>(ba_graph.variable(0));
  PoseVariableTypeAD* v1=dynamic_cast<PoseVariableTypeAD*>(ba_graph.variable(num_cameras-1));
  v1->setStatus(VariableBase::Fixed);
  
  Isometry3f x01=v0->estimate().inverse()*v1->estimate();
  float range=x01.translation().norm();

  ScaleFactorTypePtr range_factor(new ScaleFactorType);
  Eigen::Matrix<float, 1, 1> info;
  info(0,0)=1e3;
  range_factor->setInformationMatrix(info);
  range_factor->setRange(range);
  range_factor->setVariableId(0,v0->graphId());
  range_factor->setVariableId(1,v1->graphId());
  ba_graph.addFactor(range_factor);

  /*********** POPULATE GRAPHS USING PAIRWISE TRANSFORM WITH RANSAC ***********/ 
  
  // we scan each pair of variables, and we count if they observe enough common landmarks
  // since the landmarks are ordered, we can do a linear search
  // for a pair of cameras [i,j] we fill the correspondence vector
  // a correspondence is a pair of indices. They represent the position
  // in the observation vector of camera i and the position in observation vector of camera j
  // that point to rhe same entity.
  // if enough common landmarks are found, we create a correspondence vector and store it in the list
  // we also create  an epipolar factor between the two variables and we load it with the  (stored)
  // correspondences.

  epipolar::Vector3fVector from_cloud;
  epipolar::Vector3fVector to_cloud;
  
  int total_measurements=0;
  for (size_t i=0; i<camera_poses.size()-1; ++i) {
    const auto& o1=observations[i];
    int num_factors_for_pose=0;
    for (size_t j=i+1; j<camera_poses.size(); ++j) {
      from_cloud.clear();
      to_cloud.clear();
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
          from_cloud.push_back(o1.directions.at(ii).coordinates());
          to_cloud.push_back(o2.directions.at(jj).coordinates());
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
      
      CorrespondenceVector epipolar_corr;
      if (k>min_matches_for_essential) {
        Eigen::Isometry3f delta_iso=camera_poses[i]->estimate().inverse()*camera_poses[j]->estimate();
        Eigen::Isometry3f est_iso;
        std::vector<float> errors;
        cerr << endl << "Transform [ " << i << " -> " << j << "] pairs:" << k ;
        int inliers=epipolar::estimateTransformRANSAC(est_iso, errors, from_cloud, to_cloud, ransac_inlier_threshold, ransac_num_rounds, false);
        cerr << " inliers: " << inliers ;
        if (inliers<min_ransac_inliers) {
          cerr << "skip" << endl;
          continue;
        }
        for (size_t kk=0; kk<correspondences.size() && kk<errors.size(); ++kk) {
          if (fabs(errors[kk])<ransac_inlier_threshold)
            epipolar_corr.push_back(correspondences[kk]);
        }
        cerr << "OK" << endl;
        AngleAxisf rot_err(delta_iso.linear().transpose()*est_iso.linear());
        cerr << "Rot err: " <<  rot_err.angle();
        cerr << " translation: " << (delta_iso.translation().array()/est_iso.translation().array()).transpose() << endl;

        cerr << "EP ";
        EpipolarFactorType* factor=new EpipolarFactorType;
        factor->setVariableId(0,i);
        factor->setVariableId(1,j);
        factor->storeCorrespondences(epipolar_corr);
        factor->setFixed(o1.directions);
        factor->setMoving(o2.directions);
        ba_graph.addFactor(std::shared_ptr<EpipolarFactorType>(factor));

        cerr << "TR ";
        TranslationFactorPtr translation_factor(new TranslationFactor);
        //translation_essential_factor->setInformationMatrix(info);
        translation_factor->setVariableId(0,i);
        translation_factor->setVariableId(1,j);
        translation_factor->tij_essential=est_iso.translation();
        translation_graph.addFactor(translation_factor);

        cerr << "ROT_R ";
        SO3RelaxedPosePoseErrorFactorPtr orientation_relaxed_factor(new SO3RelaxedPosePoseErrorFactor);
        orientation_relaxed_factor->setMeasurement(est_iso.linear());
        orientation_relaxed_factor->setVariableId(0,i);
        orientation_relaxed_factor->setVariableId(1,j);
        orientation_graph.addFactor(orientation_relaxed_factor);

        cerr << "ROT_NL ";
        SO3PosePoseQuaternionRightErrorFactorADPtr orientation_factor(new SO3PosePoseQuaternionRightErrorFactorAD);
        orientation_factor->setMeasurement(est_iso.linear());
        orientation_factor->setVariableId(0,i);
        orientation_factor->setVariableId(1,j);
        orientation_nonlinear_graph.addFactor(orientation_factor);

        total_measurements+=k;
        ++ num_factors_for_pose;
        
      } else {
        //cerr << "poses [" << i << "," << j << "]-> " << k << "NOPE" << endl;
      }
    }
    cerr << "pose: " << i << " # factors: " << num_factors_for_pose << endl;
  }
  cerr << "num_variables: " << ba_graph.variables().size() << endl;
  cerr << "num_factors:   " << ba_graph.factors().size() << endl;
  cerr << "total_pairwise_point_measurements:   " << total_measurements << endl;
  cerr << "fill in: " << 2*(float)ba_graph.factors().size()/(float)(ba_graph.variables().size()*(ba_graph.variables().size()+1)) << endl;


  
  // instantiate and configure the solver
  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> gn(new IterationAlgorithmGN);
  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);
  
  RobustifierPolicyByTypePtr policy(new RobustifierPolicyByType);
  policy->param_factor_class_name.setValue(_FACTOR_);
  std::shared_ptr<RobustifierCauchy> robustifier(new RobustifierCauchy);
  policy->param_robustifier.setValue(robustifier);
  robustifier->param_chi_threshold.setValue(1e-3);
  //solver.param_robustifier_policies.pushBack(policy);
  gn->param_damping.setValue(0);
  std::shared_ptr<SparseBlockLinearSolverCholeskyCSparse> csparse_solver(new SparseBlockLinearSolverCholeskyCSparse);
  solver.param_verbose.setValue(true);
  solver.param_algorithm.setValue(gn);
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_linear_solver.setValue(csparse_solver);
  solver.param_max_iterations.setValue({3});
  SolverActionBasePtr act(new SolverVerboseAction);
  solver.param_actions.pushBack(act);

  /*********** SYNCHRONIZE ROTATIONS (LINEAR) ***********/ 
  
  cerr << "rotation, initial errors: " << endl;
  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f gt=gt_poses[i];
    cerr << "orientation: " << i;
    VariableSO3Relaxed* v=dynamic_cast<VariableSO3Relaxed*>(orientation_graph.variable(i));
    AngleAxisf aa(gt.linear().transpose()*v->estimate());
    cerr << "  rot_err: " <<  aa.angle() << endl;
  }

  cerr << "computing relaxed rotation guess" << endl;
  solver.setGraph(orientation_graph);
  solver.compute();

  cerr << "rotation, errors before nonlinear step: " << endl;

  /*********** SYNCHRONIZE ROTATIONS (NONLINEAR) ***********/ 
  cerr << "computinf nonlinear rotation estimates" << endl;
  solver.setGraph(orientation_nonlinear_graph);
  for (auto it: orientation_graph.variables()) {
    VariableBase* v_from=it.second;
    VariableBase* v_to=orientation_nonlinear_graph.variable(v_from->graphId());
    VariableSO3Relaxed* from=dynamic_cast<VariableSO3Relaxed*>(v_from);
    VariableSO3QuaternionRightAD* to=dynamic_cast<VariableSO3QuaternionRightAD*>(v_to);
    to->setEstimate(from->estimate());
  }

  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f gt=gt_poses[i];
    cerr << "orientation: " << i;
    VariableSO3QuaternionRightAD* v=dynamic_cast<VariableSO3QuaternionRightAD*>(orientation_nonlinear_graph.variable(i));
    AngleAxisf aa(gt.linear().transpose()*v->estimate());
    cerr << "  rot_err: " <<  aa.angle() << endl;
  }
  gn->param_damping.setValue(1);
  solver.param_max_iterations.setValue({10});
  solver.compute();

  cerr << "rotation, errors after nonlinear step: " << endl;
  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f gt=gt_poses[i];
    cerr << "orientation: " << i;
    VariableSO3QuaternionRightAD* v=dynamic_cast<VariableSO3QuaternionRightAD*>(orientation_nonlinear_graph.variable(i));
    AngleAxisf aa(gt.linear().transpose()*v->estimate());
    cerr << "  rot_err: " <<  aa.angle() << endl;
  }

  /*********** SOLVE TRANSLATIONS (LINEAR) ***********/ 

  cerr << "assembling linear translation problem" << endl;
  for (auto it: translation_graph.factors()) {
    EssentialTranslationFactor* f=dynamic_cast<EssentialTranslationFactor*>(it.second);
    if(! f)
      continue;
    int i=f->variableId(0);
    int j=f->variableId(1);
    VariableSO3QuaternionRightAD* vi=dynamic_cast<VariableSO3QuaternionRightAD*>(orientation_nonlinear_graph.variable(i));
    VariableSO3QuaternionRightAD* vj=dynamic_cast<VariableSO3QuaternionRightAD*>(orientation_nonlinear_graph.variable(j));
    f->Ri=vi->estimate();
    f->Rj=vj->estimate();
  }

  cerr << "ERRORS BEFORE" << endl;
  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f gt=gt_poses[i];
    cerr << "translation: " << i;
    VariablePoint3AD* v=dynamic_cast<VariablePoint3AD*>(translation_graph.variable(i));
    cerr << "trans: " <<  v->estimate().transpose()
         << " gt: " << gt.translation().transpose() 
         << " err: " <<  (v->estimate()-gt.translation()).transpose() << endl;
  }

  solver.param_max_iterations.setValue({1});
  solver.setGraph(translation_graph);
  gn->param_damping.setValue(0);
  std::shared_ptr<SparseBlockLinearSolverNullspace> ns_linear_solver(new SparseBlockLinearSolverNullspace);

  ns_linear_solver->param_nullspace_scales.setValue({translation_scale});
  solver.param_linear_solver.setValue(ns_linear_solver);
  solver.compute();
  

  
  cerr << "ERRORS AFTER" << endl;
  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f gt=gt_poses[i];
    cerr << "translation: " << i;
    VariablePoint3AD* v=dynamic_cast<VariablePoint3AD*>(translation_graph.variable(i));
    cerr << "trans: " <<  v->estimate().transpose()
         << " gt: " << gt.translation().transpose() 
         << " err: " <<  (v->estimate()-gt.translation()).transpose() << endl;
  }
  /*********** SOLVE BA (NONLINEAR) ***********/ 
  
  cerr << "full epipolar ba on initialized problem : " << endl;
  std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f> > init_poses;
  
  for (auto it: orientation_nonlinear_graph.variables()) {
    VariableBase* v_from_rot=it.second;
    VariableSO3QuaternionRightAD* from_rot=dynamic_cast<VariableSO3QuaternionRightAD*>(v_from_rot);
      
    VariableBase* v_from_trans=translation_graph.variable(v_from_rot->graphId());
    VariablePoint3AD* from_trans=dynamic_cast<VariablePoint3AD*>(v_from_trans);
    
    VariableBase* v_to=ba_graph.variable(v_from_rot->graphId());
    PoseVariableTypeAD* to=dynamic_cast<PoseVariableTypeAD*>(v_to);
    Isometry3f guess=Isometry3f::Identity();
    guess.linear()=from_rot->estimate();
    guess.translation()=from_trans->estimate();
    to->setEstimate(guess);
    init_poses.push_back(guess);
  }

  // solve and see what happens
  solver.setGraph(ba_graph);
  gn->param_damping.setValue(1);
  solver.param_verbose.setValue(true);
  solver.param_max_iterations.setValue({100});
  solver.param_algorithm.setValue(gn);
  solver.param_linear_solver.setValue(csparse_solver);
  solver.compute();
  ofstream os ("ba_out.txt");
  //os << "#gtx gty gtz gtqx gtqy gtqz initx inity initz initqx initqy initqz estx esty estz estqx estqy estqz" << endl;
  for (size_t i=0; i<gt_poses.size(); ++i) {
    Isometry3f init=init_poses[i];
    Isometry3f gt=gt_poses[i];
    cerr << "pose: " << i;
    PoseVariableTypeAD* v=dynamic_cast<PoseVariableTypeAD*>(ba_graph.variable(i));
    AngleAxisf aa(gt.linear().transpose()*v->estimate().linear());
    AngleAxisf aai(gt.linear().transpose()*init.linear());
    cerr << "  rot_err: " << aai.angle() << " -> " << aa.angle();
    cerr << " tras_err: (" << (init.translation()-gt.translation()).norm() <<") -> (" << (v->estimate().translation()-gt.translation()).norm() << ")" << endl;
    os << geometry3d::t2v(gt).transpose()
       << " " << geometry3d::t2v(init).transpose()
       << " " << geometry3d::t2v(v->estimate()).transpose() << endl;;
 }
}
