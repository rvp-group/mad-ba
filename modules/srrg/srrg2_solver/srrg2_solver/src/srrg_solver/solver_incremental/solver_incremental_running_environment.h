#pragma once
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_incremental/factor_graph_incremental_sorter.h>
#include <srrg_solver/utils/solver_evaluator.h>
#include <srrg_solver/solver_incremental/solver_incremental_base.h>

namespace srrg2_solver {

  struct SolverIncrementalRunningEnvironment {
    FactorGraphIncrementalReader reader;
    SolverIncrementalBasePtr  solver;
    SolverIncrementalRunningEnvironment(ParseCommandLine& cmd_line_);
    virtual void setup(std::set<VariableBase::Id>& relaxed);
    virtual void startEpochCallback(std::set<VariableBase::Id>& new_vars,
                                    std::set<VariableBase::Id>& new_factors);
    virtual void endEpochCallback();
    virtual void endDataCallback();
    virtual void run();
    virtual ~SolverIncrementalRunningEnvironment();
    bool readEpoch(std::set<VariableBase::Id>& new_vars,
                   std::set<VariableBase::Id>& new_factors);
    ParseCommandLine& cmd_line;
    ArgumentString input_file;
    ArgumentString config_file;
    ArgumentString solver_name;
    ArgumentString output_file;
    ArgumentFlag   do_tag;
    ArgumentFlag   do_finalize;
    ArgumentString benchmark_file;
    FactorGraphPtr graph, gt_graph;
    ConfigurableManager _manager;
    SolverEvaluator _evaluator;
  };

}
