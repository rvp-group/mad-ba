#pragma once
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/utils/solver_evaluator.h>
#include <srrg_solver/solver_incremental/solver_incremental_base.h>
#include <srrg_solver/solver_incremental/factor_graph_incremental_sorter.h>

namespace srrg2_solver {

  struct SolverIncrementalRunner: public Configurable, public Preemptible {
    PARAM(PropertyConfigurable_<SolverIncrementalBase>,
          solver,
          "incremental solver",
          0, 0);
    PARAM(PropertyString, input_file, "graph input filename", "", 0);
    PARAM(PropertyBool, end_epoch_tag, "if true waits for an explicit end epoch thing", false, 0);
    PARAM(PropertyBool, do_finalize, "if true when the graph is over it runs a batch optimization round", false, 0);
    PARAM(PropertyString, benchmark_file, "file where to read the gt", "", 0);
    PARAM_VECTOR(PropertyVector_<int>,
                 relaxed,
                 "ids of the vertices that are relaxed",
                 0);
    /*! Solve the optimization problem */
    void compute() override;

    void reset() override;
    virtual ~SolverIncrementalRunner();
  protected:
    void setup();
    virtual void startEpochCallback(std::set<VariableBase::Id>& new_vars,
                                    std::set<VariableBase::Id>& new_factors);
    virtual void endEpochCallback();
    virtual void endDataCallback();
    bool readEpoch(std::set<VariableBase::Id>& new_vars,
                   std::set<VariableBase::Id>& new_factors);
    FactorGraphPtr graph, gt_graph;
    SolverEvaluator _evaluator;
    FactorGraphIncrementalReader _reader;

  };

}
