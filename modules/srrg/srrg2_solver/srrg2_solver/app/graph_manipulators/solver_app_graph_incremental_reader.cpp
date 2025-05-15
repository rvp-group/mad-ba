#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>

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

#include "srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"

#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_projective/instances.h"
#include "srrg_solver/utils/instances.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"

#include <Eigen/Cholesky>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;


const std::string exe_name = "solver_app_graph_sorter";
#define LOG std::cerr << exe_name << "|"

const char* banner[] = {
  "fuck yourself",
  0
};

// ia register types
void initTypes() {
  variables_and_factors_2d_registerTypes();
  variables_and_factors_3d_registerTypes();
  variables_and_factors_projective_registerTypes();
  solver_utils_registerTypes();
  solver_registerTypes();
}

// ia THE PROGRAM
int main(int argc, char** argv) {
  initTypes();
  
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  cmd_line.parse();
  
  if (! input_file.isSet()) {
    cerr << "no input file provided, aborting" << endl;
    return 0;
  }
  std::cerr << "loding file: [" << input_file.value() << "]... " << endl;
  
  FactorGraphPtr graph(new FactorGraph);
  FactorGraphIncrementalReader reader;
  reader.setGraph(graph);
  reader.setFilePath(input_file.value());
  reader.relaxed().insert(0);
  std::set<VariableBase::Id> new_vars;
  std::set<FactorBase::Id> new_factors;
  while (reader.readEpoch(new_vars, new_factors)) {
    if (new_vars.size()) {
      cerr << "epoch: " << *new_vars.begin() << " vars: " << new_vars.size() << " factors:" << new_factors.size() << endl; 
    }
  }
  return 0;
}
