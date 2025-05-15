#pragma once
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/factor_graph_view.h"
#include <srrg_system_utils/chrono.h>

namespace srrg2_solver{
  using namespace srrg2_core;

  using Id=VariableBase::Id;
  using IdIntMap = std::map<VariableBase::Id, int>;
  using IdSet  = std::set<VariableBase::Id>;

  class SolverIncrementalBase: public Configurable {
  public:
    PARAM(PropertyString,
          gauge_type,
          "class name of the variable that can become a gauge",
          "VariableSE3QuaternionRightAD", 0);

    // access to vars that are locked (parameters)
    IdSet& parameterIds() {return _parameter_ids;}
    
    // call once before the machine runs
    virtual void setGraph(FactorGraphPtr graph_) {_graph = graph_;}
    FactorGraphPtr graph() {return _graph;}

    // call once, passing the factor pool that is new since the last epoch
    // for comments see the cpp
    virtual VariableBase::Id compute(const IdSet& factor_ids, bool force=false) = 0;
        
    virtual void finalize() = 0;
    virtual const IdSet& finalizedFactors() const = 0;

    // returns the origin of the global map in the skeleton
    VariableBase* globalGauge() { return _global_gauge; }
    virtual void computeSurroundingView(FactorGraphView& view) = 0;
    const Chrono::ChronoMap& resourceUsage() const { return _chrono_map;}
    virtual ~SolverIncrementalBase();
  protected:

    VariableBase* _global_gauge=nullptr;
    IdSet _parameter_ids;
    FactorGraphPtr _graph=nullptr;
    Chrono::ChronoMap _chrono_map;
  };

  using SolverIncrementalBasePtr=std::shared_ptr<SolverIncrementalBase>;

}
