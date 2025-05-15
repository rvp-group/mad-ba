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

#include "srrg_solver/utils/solver_evaluator.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

extern char** environ;
const std::string exe_name = environ[0];
#define LOG std::cerr << exe_name << "|"


static const char* banner[] = {
  "evaluates a  factor graph,  by registering the poses in the input graph with the corresponding poses of the gt",
  0
};


  
// ia THE PROGRAM
int main(int argc, char** argv) {
  using namespace std;
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString gt_file          (&cmd_line, "gt",    "gt-file",           "file where to read the ground truth", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",             "file where to write the output ", "");
  ArgumentString ev_mode          (&cmd_line, "m",    "mode",           "eval mode, [se3, se2, sim3]", "se3");
  ArgumentFloat cutoff          (&cmd_line, "k",    "cutoff",           "max chi to consider a factor an inlier", 0);
  cmd_line.parse();

  FactorGraphPtr graph, gt_graph;

  std::cerr << "loading file: [" << gt_file.value() << "]... ";
  gt_graph = FactorGraph::read(gt_file.value());
  std::cerr << "done, factors:" << gt_graph->factors().size() << " vars: " << gt_graph->variables().size() << std::endl;

  std::cerr << "loading file: [" << input_file.value() << "]... ";
  graph = FactorGraph::read(input_file.value());
  std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;

  std::cerr <<  "Setting evaluator ...";
  SolverEvaluator evaluator;
  if (cutoff.isSet())
    evaluator.setChiCutoff(cutoff.value());
  evaluator.setGroundTruth(*gt_graph);
  cerr << "done" << endl;
  if (ev_mode.value()=="se2") {
    evaluator.alignSE2(*graph, true);
  } else if (ev_mode.value()=="se3") {
    evaluator.alignSE3(*graph, true);
  } else if (ev_mode.value()=="sim3") {
    evaluator.alignSim3(*graph, true);
  } else {
    cerr << "unknown ev mode [" << ev_mode.value() << "]" << endl;
    return -1;
  }
  evaluator.compute(*graph);
  if (output_file.isSet()) {
    cerr << "saving transformed graph [" <<output_file.value() << "]" <<  endl;
    graph->setSerializationLevel(-1);
    graph->write(output_file.value());
  }
  return 0;
}
