#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"

#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_point_error_factor.h"

#include "srrg_solver/utils/factor_graph_initializer.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

extern char** environ;
const std::string exe_name = environ[0];
#define LOG std::cerr << exe_name << "|"


static const char* banner[] = {
  "initializes the factor graph,  attempting first a breadth first on the poses, then a local triangulation of the landmarks",
  "usage: solver_app_graph_initializer -i <input> -o <output>",
  0
};

// ia THE PROGRAM
int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentFlag incremental(&cmd_line, "s",    "incremental",           "if toggled, triggers incremental initialization");
  ArgumentFlag verbose(&cmd_line, "v",    "verbose",           "if toggled, bloats the screen with stuff");
  ArgumentString path_type          (&cmd_line, "t",    "path-type",             "type of the variable used for sorting", "VariableSE3QuaternionRightAD");

  ArgumentInt start_var          (&cmd_line, "b",    "begin", "variable to keep fixed", 0);

  cmd_line.parse();

  FactorGraphInitializer initializer;
  if (verbose.isSet())
    initializer.verbose=true;
  if (! input_file.isSet()) {
    cerr << "no input file provided, returning" << std::endl;
    return 0;
  }
  FactorGraphPtr graph;
  std::set<VariableBase::Id> parameter_ids;
  
  if (! incremental.isSet()) {
    
    std::cerr << "loading file: [" << input_file.value() << "]... ";
    graph = FactorGraph::read(input_file.value());
    std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;
    std::cerr << "initializing (batch) ...";
    auto v=graph->variable(start_var.value());
    if (!v) {
      cerr << "no variable where to start" << endl;
    }
    v->setStatus(VariableBase::Fixed);
    for (auto v_it: graph->variables()) {
      if (v_it.first!=start_var.value() && v_it.second->status()==VariableBase::Fixed) {
        initializer.parameterIds().insert(v_it.first);
      }
    }
    initializer.setGraph(*graph);
    initializer.compute();
    std::cerr << "done" << std::endl;
    int num_initialized=0;
    for (auto v_it: graph->variables()) {
      VariableBase* v=v_it.second;
      if (initializer.isInit(v)) {
        ++num_initialized;
      }
    }
    std::cerr << "total intialized: " << num_initialized << "/" << graph->variables().size() << endl;
  } else {
    graph = FactorGraphPtr(new FactorGraph);
    FactorGraphIncrementalReader reader;
    reader.setGraph(graph);
    reader.setPathType(path_type.value());
    reader.setFilePath(input_file.value());

    std::cerr << "done" << std::endl;
    initializer.setGraph(*graph);

    //process the data incrementally
    std::set<VariableBase::Id> new_vars;
    std::set<FactorBase::Id> new_factors;
    cerr << "Incremental init " << endl;
    while (reader.readEpoch(new_vars, new_factors)) {
      if (new_vars.size()) {
        cerr << "\rEpoch: " << *new_vars.begin() << " " << " v: " <<new_vars.size() << " f:" << new_factors.size();
      }
      initializer.updateGraph();
      initializer.compute();
    }
    cerr << endl;
    //final batch
    initializer.updateGraph();
    initializer.compute();
  }
  
  if (! output_file.isSet()) {
    cerr << "no output file provided, skipping output" << endl;
    return 0;
  }
  std::cerr << "writing output to file [ " << output_file.value() << "]... ";
  graph->write(output_file.value());
  cerr << " done" << endl;
  return 0;
}
