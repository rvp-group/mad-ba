#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include <srrg_system_utils/parse_command_line.h>

#include "srrg_solver/variables_and_factors/types_sfm/so3_pose_constraint_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/so3_pose_pose_error_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/variable_so3.h"

#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/robustifier.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/termination_criteria.h"

#include "sfm_common.h"
#include "srrg_geometry/epipolar.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

const char* banner[] = {
  "essential2orientations",
  "computes the absolute orientations of the essential problem passed as input",
  0};

using VariableSO3RelaxedPtr            = std::shared_ptr<VariableSO3Relaxed>;
using SO3RelaxedPosePoseErrorFactorPtr = std::shared_ptr<SO3RelaxedPosePoseErrorFactor>;

using VariableSO3Ptr = std::shared_ptr<VariableSO3QuaternionRightAD>;
using SO3PosePoseQuaternionRightErrorFactorADPtr =
  std::shared_ptr<SO3PosePoseQuaternionRightErrorFactorAD>;
using SO3RelaxedPoseConstraintFactorPtr = std::shared_ptr<SO3RelaxedPoseConstraintFactor>;
using ConstraintVectorType              = SO3RelaxedPoseConstraintFactor::ConstraintVectorType;
using ConstraintTerminationCriterion    = PerturbationNormAndConstraintViolationTerminationCriteria;
using ConstraintTerminationCriterionPtr = std::shared_ptr<ConstraintTerminationCriterion>;

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_keyframes_file(
    &cmd_line, "ik", "input-keyframes", "file where to read the keyframes ", "");
  ArgumentString input_essential_file(
    &cmd_line, "ie", "input-essential", "file where to read the essential estimate ", "");
  ArgumentString output_file(&cmd_line, "o", "output-file", "file where to save the output", "");
  ArgumentInt nonlinear_iterations(
    &cmd_line, "kn", "nonlinear-iterations", "iterations in nonlinear phase", 10);
  ArgumentInt linear_iterations(
    &cmd_line, "kl", "linear-iterations", "iterations in linear phase", 3);

  float rho, rho_max, rho_min, multiplier, damping;
  // Create an input file stream.
  ifstream in("al_parameters.txt", ios_base ::in);
  // Read data , until it is there.
  in >> rho >> rho_max >> rho_min >> multiplier >> damping;

  cmd_line.parse();

  if (!input_keyframes_file.isSet()) {
    cerr << "no input file" << endl;
    return -1;
  }

  ifstream isk(input_keyframes_file.value());
  if (!isk.good()) {
    cerr << "invalid input" << endl;
    return -1;
  }

  ifstream ise(input_essential_file.value());
  if (!ise.good()) {
    cerr << "invalid input" << endl;
    return -1;
  }

  KeyframePtrMap keyframes;
  readKeyframes(keyframes, isk);
  isk.close();

  EssentialGraph essential_graph(keyframes);
  essential_graph.read(ise);
  ise.close();

  // verify the connectivity of the graph;
  std::set<int> visited;
  for (const auto& m : essential_graph._estimates) {
    visited.insert(m.second->from);
    visited.insert(m.second->to);
  }
  std::cerr << "number of visited cameras: " << visited.size() << endl;

  std::map<int, EssentialGraph::Partition> partitions;
  essential_graph.computePartitions(partitions);
  cerr << "num_partitions: " << partitions.size() << endl;

  for (auto& p : partitions) {
    cerr << "id: " << p.first << " nodes: " << p.second.size() << endl;
  }

  if (partitions.size() != 1) {
    cerr << "essential graph is disconnected, cannot operate" << endl;
    return -1;
  }

  cerr << "assembling orientation problem (linear and nonlinear)" << endl;
  FactorGraph orientation_relaxed_graph, orientation_nonlinear_graph;
  for (auto& kf : essential_graph._keyframes) {
    {
      VariableSO3RelaxedPtr var(new VariableSO3Relaxed);
      var->setZero();
      if (kf.first == 0) {
        var->setEstimate(kf.second->gt_pose.linear());
        var->setStatus(VariableBase::Fixed);
      }
      var->setGraphId(kf.first);
      orientation_relaxed_graph.addVariable(var);
    }
    {
      VariableSO3Ptr var(new VariableSO3QuaternionRightAD);
      var->setZero();
      if (kf.first == 0) {
        var->setEstimate(kf.second->gt_pose.linear());
        var->setStatus(VariableBase::Fixed);
      }
      var->setGraphId(kf.first);
      orientation_nonlinear_graph.addVariable(var);

      SO3RelaxedPoseConstraintFactorPtr orientation_orthogonality_constraint_factor(
        new SO3RelaxedPoseConstraintFactor);
      orientation_orthogonality_constraint_factor->setVariableId(0, kf.first);
      orientation_orthogonality_constraint_factor->setRho(rho, rho_max, rho_min);
      orientation_orthogonality_constraint_factor->_constraint_type = srrg2_solver::Equality;
      ConstraintVectorType mu_initialization;
      mu_initialization = multiplier * ConstraintVectorType::Ones();
      orientation_relaxed_graph.addFactor(orientation_orthogonality_constraint_factor);
    }
  }

  for (auto& e_it : essential_graph._estimates) {
    auto& est = e_it.second;
    if (est->from != e_it.first)
      continue;
    SO3RelaxedPosePoseErrorFactorPtr orientation_relaxed_factor(new SO3RelaxedPosePoseErrorFactor);
    orientation_relaxed_factor->setMeasurement(est->relative_estimate.linear());
    orientation_relaxed_factor->setVariableId(0, est->from);
    orientation_relaxed_factor->setVariableId(1, est->to);
    orientation_relaxed_graph.addFactor(orientation_relaxed_factor);

    SO3PosePoseQuaternionRightErrorFactorADPtr orientation_nonlinear_factor(
      new SO3PosePoseQuaternionRightErrorFactorAD);
    orientation_nonlinear_factor->setMeasurement(est->relative_estimate.linear());
    orientation_nonlinear_factor->setVariableId(0, est->from);
    orientation_nonlinear_factor->setVariableId(1, est->to);
    orientation_nonlinear_graph.addFactor(orientation_nonlinear_factor);
  }

  ofstream os(output_file.value());

  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> gn(new IterationAlgorithmGN);
  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);

  gn->param_damping.setValue(damping);
  std::shared_ptr<SparseBlockLinearSolverCholeskyCSparse> csparse_solver(
    new SparseBlockLinearSolverCholeskyCSparse);
  solver.param_verbose.setValue(true);

  solver.param_algorithm.setValue(gn);
  ConstraintTerminationCriterionPtr termination_criterion(new ConstraintTerminationCriterion);
  solver.param_termination_criteria.setValue(termination_criterion);
  solver.param_linear_solver.setValue(csparse_solver);
  solver.param_max_iterations.setValue({linear_iterations.value()});
  SolverActionBasePtr act(new SolverVerboseAction);
  solver.param_actions.pushBack(act);

  cerr << "computing relaxed rotation guess" << endl;
  solver.setGraph(orientation_relaxed_graph);
  solver.compute();

  for (auto v : orientation_relaxed_graph.variables()) {
    auto v_src = dynamic_cast<VariableSO3Relaxed*>(v.second);
    if (!v_src)
      throw std::runtime_error("cast!");
    auto v_dest = dynamic_cast<VariableSO3QuaternionRightAD*>(
      orientation_nonlinear_graph.variable(v_src->graphId()));
    if (!v_dest)
      throw std::runtime_error("cast!");
    v_dest->setEstimate(v_src->estimate());
  }

  cerr << "computing nonlinear rotation guess" << endl;
  solver.param_max_iterations.setValue({nonlinear_iterations.value()});
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(orientation_nonlinear_graph);
  //  solver.compute();

  if (!os.good()) {
    cerr << "Can't open output stream" << endl;
    return -1;
  }

  for (auto v_it : orientation_nonlinear_graph.variables()) {
    auto v_dest           = dynamic_cast<VariableSO3QuaternionRightAD*>(v_it.second);
    auto& kf              = keyframes[v_dest->graphId()];
    kf->est_pose.linear() = v_dest->estimate();
  }
  for (auto& kf_it : keyframes) {
    os << *kf_it.second;
  }
}
