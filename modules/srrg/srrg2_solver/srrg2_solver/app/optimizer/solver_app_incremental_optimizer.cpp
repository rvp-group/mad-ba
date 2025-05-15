#include "srrg_solver/solver_incremental/solver_incremental_running_environment.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

std::string exe_name = environ[0];

#define LOG std::cerr << exe_name << "|"

static const char* banner[] = {
  "optimizes a graph incrementally",
  0
};


// ia THE PROGRAM
int main(int argc, char** argv) {
  exe_name=argv[0];
  ParseCommandLine cmd_line(argv, banner);
  SolverIncrementalRunningEnvironment environment(cmd_line);
  cmd_line.parse();
  std::set<VariableBase::Id> relaxed;
  relaxed.insert(0); // unlock the first node, and let th star solver do its job
  environment.setup(relaxed);
  environment.run();
  return 0;
}
