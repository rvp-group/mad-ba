#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>
#include <srrg_config/configurable_manager.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_point_error_factor.h"

#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"

#include "srrg_solver/variables_and_factors/types_projective/instances.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"

#include "srrg_solver/utils/solver_evaluator.h"
#include "srrg_solver/solver_core/solver.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

extern char** environ;
const std::string exe_name = environ[0];
#define LOG std::cerr << exe_name << "|"


static const char* banner[] = {
  "solves a factor graph from the nominal configuration and assigns to the factors the measurements that would lead to a 0 error configuration",
  0
};

struct AssignerBase{
  virtual bool assign(FactorBase* factor) = 0;
};
using AssignerBasePtr=std::unique_ptr<AssignerBase>;

template <typename FactorType_>
struct Assigner_ : public AssignerBase {
  using FactorType=FactorType_;
  bool assign(FactorBase* factor_) {
    FactorType* factor=dynamic_cast<FactorType*>(factor_);
    if (! factor)
      return false;
    do_assign(*factor);
    return true;
  }
  virtual void do_assign(FactorType& factor) = 0;
};

struct AssignerSE3PosePoseGeodesicErrorFactor: public Assigner_<SE3PosePoseGeodesicErrorFactor> {
  void do_assign(SE3PosePoseGeodesicErrorFactor& factor) override {
    VariableSE3Base* from=factor.variables().at<0>();
    VariableSE3Base* to=factor.variables().at<1>();
    factor.setMeasurement(from->estimate().inverse()*to->estimate());
  }
};

struct AssignerSE2PosePoseGeodesicErrorFactor: public Assigner_<SE2PosePoseGeodesicErrorFactor> {
  void do_assign(SE2PosePoseGeodesicErrorFactor& factor) override {
    VariableSE2Base* from=factor.variables().at<0>();
    VariableSE2Base* to=factor.variables().at<1>();
    factor.setMeasurement(from->estimate().inverse()*to->estimate());
  }
};

struct AssignerSE3PosePointOffsetErrorFactor: public Assigner_<SE3PosePointOffsetErrorFactor> {
  void do_assign(SE3PosePointOffsetErrorFactor& factor) override {
    VariableSE3Base* pose=factor.variables().at<0>();
    VariablePoint3*  point=factor.variables().at<1>();
    VariableSE3Base* offset=factor.variables().at<2>();
    Vector3f p_local=offset->estimate().inverse()*pose->estimate().inverse()*point->estimate();
    factor.setMeasurement(p_local);
  }
};

struct AssignerSE3PosePointOmniBAErrorFactor: public Assigner_<SE3PosePointOmniBAErrorFactor> {
  void do_assign(SE3PosePointOmniBAErrorFactor& factor) override {
    VariableSE3Base* pose=factor.variables().at<0>();
    VariablePoint3*  point=factor.variables().at<1>();
    VariableSE3Base* offset=factor.variables().at<2>();
    Vector3f p_local=offset->estimate().inverse()*pose->estimate().inverse()*point->estimate();
    p_local.normalize();
    factor.setMeasurement(p_local);
  }
};


struct AssignerSim3PosePoseErrorFactorAD: public Assigner_<Sim3PosePoseErrorFactorAD> {
  void do_assign(Sim3PosePoseErrorFactorAD& factor) override {
    VariableSim3Base* from=factor.variables().at<0>();
    VariableSim3Base* to=factor.variables().at<1>();
    factor.setMeasurement(from->estimate().inverse()*to->estimate());
  }
};

std::list<AssignerBasePtr> assigners;

void constructAssigners() {
  assigners.push_back(AssignerBasePtr(new(AssignerSE3PosePoseGeodesicErrorFactor)));
  assigners.push_back(AssignerBasePtr(new(AssignerSE2PosePoseGeodesicErrorFactor)));
  assigners.push_back(AssignerBasePtr(new(AssignerSE3PosePointOffsetErrorFactor)));
  assigners.push_back(AssignerBasePtr(new(AssignerSE3PosePointOmniBAErrorFactor)));
  assigners.push_back(AssignerBasePtr(new(AssignerSim3PosePoseErrorFactorAD)));
}

bool assign(FactorBase* factor) {
  for (auto& a: assigners) {
    if (a->assign(factor))
      return true;
  }
  cerr << "error in setting factor of type [ " << factor->className() << "]" << endl;
  return false;
}

// ia register types
void initTypes() {
  variables_and_factors_2d_registerTypes();
  variables_and_factors_3d_registerTypes();
  variables_and_factors_projective_registerTypes();
  solver_registerTypes();
}

  
// ia THE PROGRAM
int main(int argc, char** argv) {
  initTypes();
  constructAssigners();
  using namespace std;
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",             "file where to write the output ", "");
  ArgumentString config_file          (&cmd_line, "c",    "config-file",           "solver config file", "solver.config");
  ArgumentString solver_name          (&cmd_line, "sn",    "solver-name",           "solver name in the config", "solver");
  cmd_line.parse();

  FactorGraphPtr graph;

  std::cerr << "loading file: [" << input_file.value() << "]... ";
  graph = FactorGraph::read(input_file.value());
  std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;

  VariableBase* v0=graph->variable(0);
  if (! v0) {
    cerr << "dunno which variable to fix" << endl;
  } else {
    v0->setStatus(VariableBase::Fixed);
  }
      
  ConfigurableManager manager;
  manager.read(config_file.value());

  // ia check if solver with this specific name exists
  LOG << "loading solver [ " << config_file.value() << " ]\n";
  SolverPtr solver = manager.getByName<Solver>(solver_name.value());
  if (!solver) {
    throw std::runtime_error(exe_name + "|ERROR, cannot find solver with name [ " +
                             solver_name.value() + " ] in configuration file [ " +
                             config_file.value() + " ]");
  }
  solver->param_actions.pushBack(SolverVerboseActionPtr(new SolverVerboseAction));
  solver->setGraph(graph);
  solver->compute();

  cerr << "setting measurements... ";
  for (auto f_it: graph->factors()) {
    if (! assign(f_it.second)) {
      throw std::runtime_error("fatal");
    }
  }
  cerr << "done" << endl;

  if (output_file.isSet()) {
    cerr << "saving optimized graph" << endl;
    graph->setSerializationLevel(-1);
    graph->write(output_file.value());
  }
  return 0;
}
