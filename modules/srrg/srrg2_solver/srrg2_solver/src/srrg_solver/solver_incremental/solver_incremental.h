#pragma once
#include "solver_incremental_base.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/utils/factor_graph_initializer.h"
#include "srrg_solver/utils/factor_graph_visit.h"

namespace srrg2_solver{
  using namespace srrg2_core;

  class SolverIncremental: public SolverIncrementalBase {
  public:
    PARAM(PropertyConfigurable_<Solver>,
          global_solver,
          "solver used for optimizing the whole graph",
          0, 0);

    PARAM(PropertyConfigurable_<Solver>,
          local_solver,
          "solver used for optimizing the local portion",
          0, 0);

    
    PARAM(PropertyConfigurable_<FactorGraphVisit>,
          surrounding_visit,
          "visit to use for determining the surrounding",
          0, 0);

    PARAM(PropertyInt,
          global_step,
          "number of steps between global optimizations",
          10, 0);

    // call once before the machine runs
    virtual void setGraph(FactorGraphPtr graph_) override;

    // call once, passing the factor pool that is new since the last epoch
    VariableBase::Id compute(const IdSet& factor_ids, bool force=false) override;

    // call at the end
    void finalize() override;
    virtual const IdSet& finalizedFactors() const override;

    virtual void computeSurroundingView(FactorGraphView& view) override;
  protected:
    VariableBase* findGauge(FactorGraphInterface& view);
    int suppressNonInitialized();
    void activateVariables();
    VariableBase* _local_gauge=nullptr;
    FactorGraphInitializer _initializer;
    FactorGraphView _pending_view;
    IdSet _suppressed_vars;
    int _elapsed_global_steps=0;
    IdSet _finalized_factors;
  };

  using SolverIncrementalPtr=std::shared_ptr<SolverIncremental>;

}
