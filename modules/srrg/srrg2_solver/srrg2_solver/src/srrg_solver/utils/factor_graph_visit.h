#pragma once
#include "factor_graph_visit_policy.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class FactorGraphVisit : public Configurable {
  public:
    using BaseType = Configurable;
    using ThisType = FactorGraphVisit;
    PARAM(PropertyFloat, max_cost,  "maximum cost when to stop the expansion", -1.f, 0);
    PARAM_VECTOR(
      PropertyConfigurableVector_<FactorGraphVisitPolicyBase>,
      cost_policies,
      "the policies used to compute the cost of the factors during the visit",
      0);

    float cost(FactorBase& factor, int var_pos, int parent_pos);

    inline void setGraph(FactorGraphInterface& graph_) {
      _graph = &graph_;
    }

    inline std::set<VariableBase::Id>& sources() { return _sources; }

    inline std::set<VariableBase::Id>& tainted() { return _tainted;}

    void compute();
    inline VariableVisitEntryContainer& entries() {
      return _entries;
    }

    float averageCost();
    float maxCost();
    
  protected:
    FactorGraphVisitEntryQueue _queue;
    std::set<VariableBase::Id> _sources;
    std::set<VariableBase::Id> _tainted;
    FactorGraphInterface* _graph                  = 0;
    VariableVisitEntryContainer _entries;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphVisitPtr = std::shared_ptr<FactorGraphVisit>;
  
} // namespace srrg2_solver
