#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_2d/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/variables_and_factors/types_projective/all_types.h"
#include <iostream>

namespace srrg2_solver{
  using namespace srrg2_core;

  template <typename FactorType_>
  void SolverEvaluator::align(FactorGraphView& view){
    using FactorType   = FactorType_;
    using VariableType = typename FactorType::VariableTupleType::template VariableTypeAt<0>;
    Point3fVectorCloud fixed_cloud, moving_cloud;
    CorrespondenceVector correspondences;
    IdSet variable_ids;
    for (auto f_it: view.factors()){
      FactorType* f=dynamic_cast<FactorType_*>(f_it.second);
      if (! f)
        continue;
      for (int i=0; i<f->numVariables(); ++i)
        variable_ids.insert(f->variableId(0));
    }

    int k=0;
    for (auto v_id: variable_ids) {
      VariableBase* v_fix_=_gt->variable(v_id);
      VariableType* v_fix=dynamic_cast<VariableType*>(v_fix_);
      assert(v_fix);
      Point3f p;
        
      if (v_fix->estimate().translation().rows()==2) {
        p.coordinates().head<2>()=v_fix->estimate().translation();
      } else {
        p.coordinates()=v_fix->estimate();
      }
      fixed_cloud.push_back(p);
      VariableBase* v_mov_=_gt->variable(v_id);
      VariableType* v_mov=dynamic_cast<VariableType*>(v_mov_);
      assert(v_mov);
      if (v_mov->estimate().translation().rows()==2) {
        p.coordinates().head<2>()=v_mov->estimate().translation();
      } else {
        p.coordinates()=v_mov->estimate();
      }
      moving_cloud.push_back(p);
      correspondences.push_back(Correspondence(k,k));
      ++k;
    }

    Solver solver;
    std::shared_ptr<SE3Point2PointErrorFactorCorrespondenceDriven>
      factor(new SE3Point2PointErrorFactorCorrespondenceDriven);
    factor->setFixed(fixed_cloud);
    factor->setMoving(moving_cloud);
    factor->setCorrespondences(correspondences);
    VariableSE3QuaternionRight * v=new VariableSE3QuaternionRight;
    FactorGraph opt_graph;
    v->setGraphId(0);
    opt_graph.addVariable(std::shared_ptr<VariableBase>(v));
    factor->setVariableId(0,0);
    factor->setInformationMatrix(Matrix3f::Identity());
    opt_graph.addFactor(factor);
    solver.param_max_iterations.pushBack(100);
    solver.param_termination_criteria.setValue(nullptr);
    v->setEstimate(Isometry3f::Identity());
    solver.compute();
    std::cerr << solver.iterationStats() << std::endl;
    
  }

  void alignSE2(FactorGraphView& view) {
    align<
  }
  void alignSE3(FactorGraphView& view);
  void alignSim3(FactorGraphView& view);

}
