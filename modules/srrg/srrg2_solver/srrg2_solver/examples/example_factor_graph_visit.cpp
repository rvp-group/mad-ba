#include <srrg_system_utils/parse_command_line.h>
#include <srrg_config/configurable_manager.h>
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/variable.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/utils/factor_graph_visit.h"
#include "srrg_solver/utils/factor_graph_visit_policy.h"
#include "srrg_solver/utils/factor_graph_visit_cost.h"

using namespace srrg2_core;
using namespace srrg2_solver;

static const char* banner[] = {
  "performs a visit of the factor graph, given a set of cost functions morte",
  0
};

ConfigurableManager manager;


int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",         "file where to read the input ", "");
  ArgumentString config_file          (&cmd_line, "c",    "config-file",       "file where to read the config ", "");

  ArgumentString config_gen          (&cmd_line, "j",    "config-gen",         "file where to write the config ", "");
  cmd_line.parse();

  if (!config_file.isSet()) {
    manager.create<FactorGraphVisit>("visit");
    auto cost = manager.create<FactorGraphVisitCostUniform>("uniform_cost");
    auto policy = manager.create<FactorGraphVisitPolicyBase>("all_policy");
    policy->param_cost_function.setValue(cost);
  }

  FactorGraphVisitPtr visit;
  if (config_gen.isSet()) {
    manager.write(config_gen.value());
    return 0;
  } else {
    manager.read(config_file.value());
    visit=manager.getByName<FactorGraphVisit>("visit");
    if (! visit) {
      std::cerr << "unable to retrieve an object named [visit] from config file, aborting" << std::endl;
    }
  }
  if (! input_file.isSet()) {
    std::cerr << "please provide a graph input for the visit" << std::endl;
    return 0;
  }

  std::cerr << "loading graph [" << input_file.value() << "]... ";
  FactorGraphPtr graph=FactorGraph::read(input_file.value());
  std::cerr << "done" << std::endl;
  std::cerr << "performing visit" << std::endl;
  visit->setGraph(*graph);
  visit->tainted().insert(2500);
  for (auto it: graph->variables()) {
    VariableSE3Base* v=dynamic_cast<VariableSE3Base*>(it.second);
    if (v) {
      if (v->graphId()==2500)
        continue;
      std::cerr << "Visit by ID: " << v->graphId() << ": ";
      visit->sources().clear();
      visit->sources().insert(v->graphId());
      visit->compute();
      std::cerr << "average_cost: " << visit->averageCost() << std::endl;
      return 0;
    }
  }
  std::cerr << "done" << std::endl;
  
}
