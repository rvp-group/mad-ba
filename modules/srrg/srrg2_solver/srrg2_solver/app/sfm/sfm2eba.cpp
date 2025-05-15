#include <srrg_system_utils/parse_command_line.h>
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_epipolar_ba_left_factor.h"
#include "srrg_solver/variables_and_factors/types_sfm/se3_pairwise_scale_error_factor_ad.h"
#include "srrg_geometry/epipolar.h"
#include "sfm_common.h"

#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/iteration_algorithm_lm.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "srrg_solver/solver_core/robustifier.h"
#include "srrg_solver/solver_core/factor_graph.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

const char * banner []= {
  "sfm2eba",
  "runs BA on epipolar only constraints",
  0
};

using PoseVariableType=VariableSE3QuaternionLeft;
using PoseVariableTypeAD=VariableSE3QuaternionLeftAD;
using ScaleFactorType=SE3PairwiseScaleErrorFactorQuaternionLeftAD;
using ScaleFactorTypePtr=std::shared_ptr<ScaleFactorType>;
using EpipolarFactorType=SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven;
using EpipolarFactorTypePtr=std::shared_ptr<EpipolarFactorType>;

#define _FACTOR_ "SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven"
using namespace epipolar;

struct VariableKeyframe: public VariableSE3QuaternionLeftAD{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Point3fVectorCloud directions;
};

using VariableKeyframePtr=std::shared_ptr<VariableKeyframe>;

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentInt min_common_landmarks          (&cmd_line, "l",    "min-common-landmarks",           "min-landmarks to try computing essential", 30);
  ArgumentInt max_iterations          (&cmd_line, "k",    "max-iterations",           "run the solver at most n rounds", 30);
  
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

  FactorGraph eba_graph;
  cerr << "populating variables" << endl;
  for (auto& kf_it: keyframes) {
    Keyframe& kf=*kf_it.second;
    std::shared_ptr<VariableKeyframe> var_kf(new VariableKeyframe);
    var_kf->setGraphId(kf.id);
    var_kf->setEstimate(kf.est_pose);
    var_kf->directions.resize(kf.measurements.size());
    int k=0;
    for (const auto& m: kf.measurements) {
      var_kf->directions[k].coordinates()=m.measurement;
      ++k;
    }

    // fix first variable
    if (! kf.id)
      var_kf->setStatus(VariableBase::Fixed);
    
    eba_graph.addVariable(var_kf);
  };
  
  cerr << "constructing eba factors" << endl;
  for (auto& last_it: keyframes) {
    auto& last_keyframe=last_it.second;
    for (auto past_it=keyframes.begin(); past_it->first <last_it.first ; ++past_it) {
      auto& past_keyframe=past_it->second;
      std::set<int> common_landmarks;
      std::set_intersection(last_keyframe->observed_landmark_ids.begin(),
                            last_keyframe->observed_landmark_ids.end(),
                            past_keyframe->observed_landmark_ids.begin(),
                            past_keyframe->observed_landmark_ids.end(),
                            std::inserter(common_landmarks, common_landmarks.begin()));
      int num_matches=common_landmarks.size();
      if (num_matches <= min_common_landmarks.value()) {
        continue;
      }
      
      cerr << "MATCHING: " << last_it.first << " " << past_it->first << " " << num_matches << endl;
      CorrespondenceVector correspondences(num_matches);
      
      int ip=0;
      int il=0;
      int dest_k=0;
      for (const auto& landmark_id: common_landmarks) {
        while (past_keyframe->measurements[ip].id != landmark_id)
          ++ip;
        while (last_keyframe->measurements[il].id != landmark_id)
          ++il;
        if (past_keyframe->measurements[ip].id != last_keyframe->measurements[il].id) {
          throw std::runtime_error("id mismatch in essential");
        }
        correspondences[dest_k].fixed_idx=ip;
        correspondences[dest_k].moving_idx=il;
        ++dest_k;
      }

      EpipolarFactorTypePtr factor(new EpipolarFactorType);
      factor->setVariableId(0,past_keyframe->id);
      factor->setVariableId(1,last_keyframe->id);
      factor->storeCorrespondences(correspondences);

      VariableKeyframe* v_from=dynamic_cast<VariableKeyframe*>(eba_graph.variable(past_keyframe->id));
      VariableKeyframe* v_to=dynamic_cast<VariableKeyframe*>(eba_graph.variable(last_keyframe->id));
      
      factor->setFixed(v_from->directions);
      factor->setMoving(v_to->directions);
      eba_graph.addFactor(std::shared_ptr<EpipolarFactorType>(factor));
    }
  }

  cerr << "adding translation scale constraint" << endl;

  int last_var_id=keyframes.rbegin()->first;
  // compute the distance between the first and last pose and impose the scale
  PoseVariableTypeAD* v0=dynamic_cast<PoseVariableTypeAD*>(eba_graph.variable(0));
  PoseVariableTypeAD* v1=dynamic_cast<PoseVariableTypeAD*>(eba_graph.variable(last_var_id));
  cerr <<"lv_id: "<< last_var_id << endl;
  //v1->setStatus(VariableBase::Fixed);
  
  Isometry3f x01=v0->estimate().inverse()*v1->estimate();
  float range=x01.translation().norm();

  ScaleFactorTypePtr range_factor(new ScaleFactorType);
  Eigen::Matrix<float, 1, 1> info;
  info(0,0)=1e3;
  range_factor->setInformationMatrix(info);
  range_factor->setRange(range);
  range_factor->setVariableId(0,v0->graphId());
  range_factor->setVariableId(1,v1->graphId());
  eba_graph.addFactor(range_factor);

  cerr << "computing solution" << endl;
  Solver solver;
  // configure and run the solver
  std::shared_ptr<IterationAlgorithmGN> gn(new IterationAlgorithmGN);
  gn->param_damping.setValue(0);

  std::shared_ptr<IterationAlgorithmLM> lm(new IterationAlgorithmLM);
  solver.param_algorithm.setValue(gn);

  std::shared_ptr<SparseBlockLinearSolverCholeskyCSparse> csparse_solver(new SparseBlockLinearSolverCholeskyCSparse);
  solver.param_linear_solver.setValue(csparse_solver);

  
  // add a robustifier
  RobustifierPolicyByTypePtr policy(new RobustifierPolicyByType);
  policy->param_factor_class_name.setValue(_FACTOR_);
  std::shared_ptr<RobustifierCauchy> robustifier(new RobustifierCauchy);
  policy->param_robustifier.setValue(robustifier);
  robustifier->param_chi_threshold.setValue(1e-2);
  solver.param_robustifier_policies.pushBack(policy);
  solver.param_termination_criteria.setValue(nullptr);
  
  // make it verbose
  SolverActionBasePtr act(new SolverVerboseAction);
  solver.param_actions.pushBack(act);
  solver.param_max_iterations.pushBack(max_iterations.value());
  solver.setGraph(eba_graph);
  solver.param_verbose.setValue(true);
  solver.compute();

  for (auto v_it: eba_graph.variables()) {
    VariableSE3QuaternionRightAD* v_pose=dynamic_cast<VariableSE3QuaternionRightAD*>(v_it.second);
    if (v_pose) {
      auto p_it=keyframes.find(v_pose->graphId());
      if (p_it!=keyframes.end())
        keyframes[v_pose->graphId()]->est_pose=v_pose->estimate();
    }
  }

  ofstream os(output_file.value());
  if (! os.good()) {
    cerr << "Can't open output stream" << endl;
    return -1;
  }
  for (auto& kf_it: keyframes) {
    os << *kf_it.second;
  }

}
