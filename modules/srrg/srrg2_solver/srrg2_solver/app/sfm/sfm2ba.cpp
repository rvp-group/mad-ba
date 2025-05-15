#include <srrg_system_utils/parse_command_line.h>
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"
#include "srrg_geometry/epipolar.h"
#include "sfm_common.h"

#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

const char * banner []= {
  "sfm2essential",
  "matches pairwise frames and constructs the essential graph",
  0
};

using VariablePoint3ADPtr = std::shared_ptr<VariablePoint3AD>;
using VariableSE3QuaternionRightADPtr = std::shared_ptr<VariableSE3QuaternionRightAD>;
using SE3PosePointOmniBAErrorFactorPtr = std::shared_ptr<SE3PosePointOmniBAErrorFactor>;

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");

  ArgumentInt structure_iterations          (&cmd_line, "ks",    "structure-iterations",           "num_iterations for landmark init", 110);
  ArgumentInt structure_and_motion_iterations          (&cmd_line, "km",    "sfm-iterations",      "num_iterations for full BA", 10);
  cmd_line.parse();

  if (! input_file.isSet()) {
    cerr << "no input file" << endl;
    return -1;
  }

  ifstream is(input_file.value());
  if (! is.good()) {
    cerr << "invalid input" << endl;
    return -1;
  }
  KeyframePtrMap keyframes;
  readKeyframes(keyframes, is);
  is.close();
  
  FactorGraph ba_graph;
  VariableSE3QuaternionRightADPtr sensor_offset(new VariableSE3QuaternionRightAD);
  sensor_offset->setGraphId(10000000);
  sensor_offset->setZero();
  sensor_offset->setStatus(VariableBase::Fixed);
  ba_graph.addVariable(sensor_offset);
  Eigen::Matrix3f info;
  info.setIdentity();
  info*=10;
  for (auto& kf_it: keyframes) {
    auto& kf=kf_it.second;
    VariableSE3QuaternionRightADPtr camera_v(new VariableSE3QuaternionRightAD);

    camera_v->setGraphId(kf->id);
    camera_v->setEstimate(kf->est_pose);
    camera_v->setStatus(VariableBase::Fixed);
    ba_graph.addVariable(camera_v);
    for (auto& m: kf->measurements) {
      VariablePoint3AD* v_point =dynamic_cast<VariablePoint3AD*>(ba_graph.variable(m.id));
      if (!v_point) {
        v_point=new VariablePoint3AD;
        v_point->setZero();
        v_point->setGraphId(m.id);
        ba_graph.addVariable(VariablePoint3ADPtr(v_point));
      }
      
      SE3PosePointOmniBAErrorFactorPtr factor(new SE3PosePointOmniBAErrorFactor);
      factor->setVariableId(0,camera_v->graphId());
      factor->setVariableId(1,v_point->graphId());
      factor->setVariableId(2,sensor_offset->graphId());
      factor->setMeasurement(m.measurement);
      factor->setInformationMatrix(info);
      ba_graph.addFactor(factor);
      
    }
  }

  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> gn(new IterationAlgorithmGN);
  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);

  RobustifierPolicyByTypePtr policy(new RobustifierPolicyByType);
  policy->param_factor_class_name.setValue("SE3PosePointOmniBAErrorFactor");
  std::shared_ptr<RobustifierCauchy> robustifier(new RobustifierCauchy);
  policy->param_robustifier.setValue(robustifier);
  robustifier->param_chi_threshold.setValue(100);
  solver.param_robustifier_policies.pushBack(policy);

  gn->param_damping.setValue(1);
  solver.param_algorithm.setValue(lm);

  std::shared_ptr<SparseBlockLinearSolverCholeskyCSparse> csparse_solver(new SparseBlockLinearSolverCholeskyCSparse);
  solver.param_linear_solver.setValue(csparse_solver);

  solver.param_max_iterations.pushBack(structure_iterations.value());
  SolverActionBasePtr act(new SolverVerboseAction);
  solver.param_actions.pushBack(act);

  cerr<< "***** LANDMARK ITERATIONS *****" << endl;
  solver.setGraph(ba_graph);
  solver.compute();

  
  cerr<< "***** FULL BA ITERATIONS *****" << endl;
  solver.param_max_iterations.value()[0]=structure_and_motion_iterations.value();
  for (auto v_it: ba_graph.variables()) {
    VariableSE3QuaternionRightAD* v=dynamic_cast<VariableSE3QuaternionRightAD*>(v_it.second);
    if (!v)
      continue;
    // leave 'em fixed
    if (v->graphId()==sensor_offset->graphId() || v->graphId()==0)
      continue;
    v->setStatus(VariableBase::Active);
  }
  solver.setGraph(ba_graph);
  solver.compute();

  std::map<int, VariablePoint3AD*> landmarks;
  for (auto v_it: ba_graph.variables()) {
    VariableSE3QuaternionRightAD* v_pose=dynamic_cast<VariableSE3QuaternionRightAD*>(v_it.second);
    if (v_pose) {
      auto p_it=keyframes.find(v_pose->graphId());
      if (p_it!=keyframes.end())
        keyframes[v_pose->graphId()]->est_pose=v_pose->estimate();
    }
    VariablePoint3AD* v_landmark=dynamic_cast<VariablePoint3AD*>(v_it.second);
    if (v_landmark) {
      landmarks[v_landmark->graphId()]=v_landmark;
    }
  }
  ofstream os(output_file.value());
  if (! os.good()) {
    cerr << "Can't open output stream" << endl;
    return -1;
  }

  for (auto& l_it: landmarks) {
    os << "LANDMARK: " << l_it.first << " " << l_it.second->estimate().transpose() << endl;
  }
  for (auto& kf_it: keyframes) {
    os << *kf_it.second;
  }
}
