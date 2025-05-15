#pragma once
#include "factor_graph_initializer_rule.h"
#include "factor_graph_visit_entry.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include <list>

namespace srrg2_solver {
  using namespace srrg2_core;

  struct FactorGraphInitializer : public Configurable {
    using BaseType = Configurable;
    using ThisType = FactorGraphInitializer;
    PARAM(PropertyInt, level, "level of factors to use in initialization", -1, 0);
    PARAM(PropertyInt, max_cost, "max diameter to choose for initialization", -1, 0);
    FactorGraphInitializer();
    void setGraph(FactorGraphInterface& graph_);
    void updateGraph();
    void compute();

    FactorGraphInitializerRulePtrContainer _rules;
    VariableVisitEntryContainer _entries;
    FactorGraphInterface* _graph = 0;
    FactorGraphVisitEntryQueue _queue;
    bool isInit(VariableBase* variable);
    const IdSet& initializedVariables() const {return _initialized_variables;}
    bool verbose=false;
    IdSet& parameterIds() {return _parameter_ids;}
    float maxCost();
  protected:
    bool initVariable(VariableBase* variable);
    IdSet _initialized_variables;
    float getCost(VariableBase* v);
    IdSet _parameter_ids;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphInitializerPtr=std::shared_ptr<FactorGraphInitializer>;
} // namespace srrg2_solver
