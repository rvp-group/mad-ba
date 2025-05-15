#include <srrg_system_utils/parse_command_line.h>
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"

#include "essential_translation_factor.h"
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>

#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_nullspace.h"
#include "srrg_solver/solver_core/robustifier.h"

#include "srrg_geometry/epipolar.h"
#include "sfm_common.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

                                                                   
const char * banner []= {
  "essential2orientations",
  "computes the absolute orientations of the essential problem passed as input",
  0
};

using VariableTranslation = VariablePoint3AD;
using VariableTranslationPtr = std::shared_ptr<VariableTranslation>;
using TranslationFactor = EssentialTranslationFactor;
using TranslationFactorPtr = std::shared_ptr<TranslationFactor>;


int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_keyframes_file          (&cmd_line, "ik",    "input-keyframes",             "file where to read the keyframes ", "");
  ArgumentString input_essential_file          (&cmd_line, "ie",    "input-essential",             "file where to read the essential estimate ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentFloat  scale (&cmd_line, "s",    "scale",           "scale of the solution, if it does not make sense give it a negative value", 100);
  cmd_line.parse();

  if (! input_keyframes_file.isSet()) {
    cerr << "no input file" << endl;
    return -1;
  }

  ifstream isk(input_keyframes_file.value());
  if (! isk.good()) {
    cerr << "invalid input [keyframes]" << endl;
    return -1;
  }

  ifstream ise(input_essential_file.value());
  if (! ise.good()) {
    cerr << "invalid input [essential]" << endl;
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
  for (const auto& m: essential_graph._estimates) {
    visited.insert(m.second->from);
    visited.insert(m.second->to);
  }
  std::cerr << "number of visited cameras: " << visited.size() << endl;
  if (visited.size()!=keyframes.size()) {
    cerr << "essential graph is disconnected, cannot operate" << endl;
  }
  
  
  cerr << "assembling translation problem (linear)" << endl;
  FactorGraph translation_graph;
  for (auto& kf: essential_graph._keyframes) {
    VariableTranslationPtr translation_var(new VariableTranslation);
    if (kf.first==0)
      translation_var->setStatus(VariableBase::Fixed);
    translation_var->setGraphId(kf.first);
    translation_var->setZero();
    translation_graph.addVariable(translation_var);
  }

  for (auto& e_it: essential_graph._estimates) {
    auto& est=e_it.second;
    if (est->from!=e_it.first)
      continue;
    TranslationFactorPtr translation_factor(new TranslationFactor);
    //translation_essential_factor->setInformationMatrix(info);
    translation_factor->setVariableId(0,est->from);
    translation_factor->setVariableId(1,est->to);
    translation_factor->tij_essential=est->relative_estimate.translation();
    translation_factor->Ri=keyframes[est->from]->est_pose.linear();
    translation_factor->Rj=keyframes[est->to]->est_pose.linear();
    translation_graph.addFactor(translation_factor);
  }

  Solver solver;
  std::shared_ptr<IterationAlgorithmGN> gn(new IterationAlgorithmGN);
  
  gn->param_damping.setValue(0);
  solver.param_verbose.setValue(true);
  solver.param_algorithm.setValue(gn);
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.setValue({1});
  solver.setGraph(translation_graph);
  gn->param_damping.setValue(0);
  std::shared_ptr<SparseBlockLinearSolverNullspace> ns_linear_solver(new SparseBlockLinearSolverNullspace);
  ns_linear_solver->param_nullspace_scales.setValue({-100});

  solver.param_linear_solver.setValue(ns_linear_solver);
  SolverActionBasePtr act(new SolverVerboseAction);
  solver.param_actions.pushBack(act);

  cerr << "computing initial translations" << endl;
  solver.setGraph(translation_graph);
  solver.compute();

  for (auto v_it: translation_graph.variables()){
    auto v=dynamic_cast<VariableTranslation*>(v_it.second);
    keyframes[v->graphId()]->est_pose.translation()=v->estimate();
  }

  ofstream os(output_file.value());
  if (! os.good()) {
    cerr << "Can't open output stream" << endl;
    return -1;
  }

  if (output_file.isSet()) {
    ofstream osf(output_file.value());
    for (const auto& kf: keyframes)
      osf << *(kf.second);
  }
}
