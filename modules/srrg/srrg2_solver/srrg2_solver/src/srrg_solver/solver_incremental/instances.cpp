#include "instances.h"
#include "solver_incremental.h"
#include "factor_graph_incremental_sorter.h"
#include "solver_incremental_runner.h"

namespace srrg2_solver {
  void solver_incremental_registerTypes() {
    BOSS_REGISTER_CLASS(SolverIncremental);
    BOSS_REGISTER_CLASS(SolverIncrementalRunner);
    BOSS_REGISTER_CLASS(EndEpoch);
  }
} // namespace srrg2_solver
