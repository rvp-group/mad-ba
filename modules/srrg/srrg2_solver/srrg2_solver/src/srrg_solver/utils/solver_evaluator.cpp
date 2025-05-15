#include "solver_evaluator.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/variables_and_factors/types_2d/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/variables_and_factors/types_projective/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_point2point_error_factor.h"

#include <srrg_geometry/geometry_defs.h>
#include <iostream>

namespace srrg2_solver {
  using namespace std;
  
  void SolverEvaluator::setGroundTruth(FactorGraphInterface& gt_) {
    _gt=&(gt_);
  }

  void SolverEvaluator::compute(FactorGraphInterface& view_) {
    if (_gt == nullptr)
      throw std::runtime_error("SolverEvaluator::setEvaluationView, no gt set");
    IdSet src_ids;
    for (auto src_it: view_.variables()) {
      auto src_id=src_it.first;
      VariableBase* src=src_it.second;
      VariableBase* dest=_gt->variable(src_id);
      if (! dest) {
        throw std::runtime_error(std::string("var ") + std::to_string(src_id) + " in gt graph, skipping");  }
      dest->push();
      dest->copyEstimate(*src);
      src_ids.insert(src_id);
    }
    _eval_view.clear();
    _eval_view.addVariables(*_gt, src_ids);

    double chi=0;
    int num_errors=0;
    int num_inliers=0;
    int num_suppressed=0;
    int num_disabled=0;
    for (auto f_it: _eval_view.factors()) {
      FactorBase* f=f_it.second;
      //all vars should be active
      bool is_disabled=false;
      for (int i=0; i<f->numVariables(); ++i) {
        int v_id=f->variableId(i);
        VariableBase* v=_gt->variable(v_id);
        if (v->status()==VariableBase::NonActive) {
          is_disabled=true;
          break;
        }
      }
      if (is_disabled){
        ++num_disabled;
        continue;
      }
      
      f->compute(true, true);
      switch(f->stats().status) {
      case FactorStats::Error:
        ++num_errors;
        break;
      case FactorStats::Inlier:
        if (f->stats().chi< _chi_cutoff) {
          chi+=f->stats().chi;
          ++num_inliers;
        } else {
          ++num_suppressed;
        }
        break;
      case FactorStats::Suppressed:
        ++num_suppressed;
        break;
      default:;
      }
    }
    std::cerr << "STATS|"
              <<" #fac: " << _eval_view.factors().size()
              <<" #inl: " << num_inliers
              <<" #dis: " << num_disabled
              <<" #err: " << num_errors
              <<" inl_chi: " << chi 
              <<" chi/inl: " << chi/num_inliers << std::endl;

    for (auto src_it: view_.variables()) {
      auto src_id=src_it.first;
      VariableBase* dest=_gt->variable(src_id);
      dest->pop();
    }
  }

  void getTransform(Isometry3f& dest, const Similiarity3f& src) {
    dest.linear()=src.linear();
    dest.translation()=src.translation();
  }
  
  void getTransform(Isometry2f& dest, const Similiarity3f& src) {
    dest.linear()=src.linear().block<2,2>(0,0);
    dest.translation()=src.translation().head<2>();
  }

  void getTransform(Similiarity3f& dest, const Similiarity3f& src) {
    dest=src;
  }

  template <typename VariableType_, typename TransformType_>
  bool applyTransform_(VariableBase* moved_, TransformType_& transform_) {
    VariableType_* moved=dynamic_cast<VariableType_*>(moved_);
    if (! moved)
      return false;
    moved->setEstimate(transform_ * moved->estimate());
    return true;
  }

  bool applyTransform(VariableBase* v,  Eigen::Isometry2f& iso) {
    if (v->status()==VariableBase::Fixed)
      return false;
    if (applyTransform_<VariablePoint2,  Eigen::Isometry2f>(v, iso))
      return true;
    if (applyTransform_<VariableSE2Base, Eigen::Isometry2f>(v, iso))
      return true;
    return false;
  }

  bool applyTransform(VariableBase* v, Eigen::Isometry3f& iso) {
    if (v->status()==VariableBase::Fixed) {
      return false;
    }
    if (applyTransform_<VariablePoint3, Eigen::Isometry3f>(v, iso))
      return true;
    if (applyTransform_<VariableSE3Base, Eigen::Isometry3f>(v, iso))
      return true;
    return false;
  }

  bool applyTransform(VariableBase* v, Similiarity3f& iso) {
    if (v->status()==VariableBase::Fixed)
      return false;
    if (applyTransform_<VariablePoint3, Similiarity3f>(v, iso))
      return true;
    if (applyTransform_<VariableSim3Base, Similiarity3f>(v, iso))
      return true;
    return false;
  }


  template <typename FactorType_>
  void SolverEvaluator::align(FactorGraphInterface& view, bool adjust){
    using FactorType   = FactorType_;
    using VariableType = typename FactorType::VariableTupleType::template VariableTypeAt<0>;
    using EstimateType = typename VariableType::EstimateType;
    Point3fVectorCloud fixed_cloud, moving_cloud;
    CorrespondenceVector correspondences;
    IdSet variable_ids;
    for (auto f_it: view.factors()){
      FactorType* f=dynamic_cast<FactorType_*>(f_it.second);
      if (! f)
        continue;
      for (int i=0; i<f->numVariables(); ++i)
        variable_ids.insert(f->variableId(i));
    }

    int k=0;
    for (auto v_id: variable_ids) {
      VariableBase* v_fix_=_gt->variable(v_id);
      VariableType* v_fix=dynamic_cast<VariableType*>(v_fix_);
      if (! v_fix) {
        cerr << "bokkeeping error, var " << v_id << " not in input graph" << endl;
        continue;
      }
      assert(v_fix);
      Point3f p_fix;

      if constexpr(EstimateType::MatrixType::ColsAtCompileTime==3) {
        p_fix.coordinates().head<2>()=v_fix->estimate().translation();
      } else {
        p_fix.coordinates()=v_fix->estimate().translation();
      }

      Point3f p_mov;
      VariableBase* v_mov_=view.variable(v_id);
      VariableType* v_mov=dynamic_cast<VariableType*>(v_mov_);
      if (! v_mov) {
        cerr << "bokkeeping error, var " << v_id << " not in gt graph" << endl;
        continue;
      }
      if constexpr(EstimateType::MatrixType::ColsAtCompileTime==3) {
        p_mov.coordinates().head<2>()=v_mov->estimate().translation();
      } else {
        p_mov.coordinates()=v_mov->estimate().translation();
      }

      fixed_cloud.push_back(p_fix);
      moving_cloud.push_back(p_mov);
      correspondences.push_back(Correspondence(k,k));
      ++k;
    }
    cerr << "cloud complete" << endl;
    
    FactorGraph opt_graph;
    std::shared_ptr<VariableSim3QuaternionRight> v(new VariableSim3QuaternionRight);
    v->setGraphId(0);
    v->setZero();
    opt_graph.addVariable(v);
    v->setStatus(VariableBase::Active);
    std::shared_ptr<Sim3Point2PointErrorFactorCorrespondenceDriven>
      factor(new Sim3Point2PointErrorFactorCorrespondenceDriven);
    factor->setFixed(fixed_cloud);
    factor->setMoving(moving_cloud);
    factor->setCorrespondences(correspondences);
    factor->setVariableId(0,0);
    factor->setInformationMatrix(Matrix3f::Identity());
    opt_graph.addFactor(factor);

    Solver solver;
    solver.param_max_iterations.pushBack(100);
    auto ialg=std::shared_ptr<IterationAlgorithmGN>(new IterationAlgorithmGN);
    ialg->param_damping.setValue(0);
    solver.param_algorithm.setValue(ialg);
    solver.param_termination_criteria.setValue(nullptr);
    solver.setGraph(opt_graph);
    solver.compute();
    
    cerr << "EVALUATOR ALIGN| FIRST " << *solver.iterationStats().begin() << endl;
    cerr << "EVALUATOR ALIGN| LAST " << *solver.iterationStats().rbegin() << endl;

    if (adjust) {
      EstimateType  transform;
      getTransform(transform, v->estimate());
      cerr << "transform: " << endl << transform.matrix() << endl;
      //apply the transformation to the variables in the view
      for (auto v_it: view.variables()) {
        VariableBase* v_mov=v_it.second;
        applyTransform(v_mov, transform);
      }
    }    
  }

  void SolverEvaluator::alignSE2(FactorGraphInterface& view, bool adjust) {
    align<SE2PosePoseGeodesicErrorFactor>(view, adjust);
  }

  void SolverEvaluator::alignSE3(FactorGraphInterface& view, bool adjust) {
    align<SE3PosePoseGeodesicErrorFactor>(view, adjust);
  }

  void SolverEvaluator::alignSim3(FactorGraphInterface& view, bool adjust) {
    align<Sim3PosePoseErrorFactorAD>(view, adjust);
  }

}
