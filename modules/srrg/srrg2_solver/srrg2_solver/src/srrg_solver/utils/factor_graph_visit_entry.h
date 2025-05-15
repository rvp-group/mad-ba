#pragma once
#include "srrg_solver/solver_core/variable.h"
#include "srrg_solver/solver_core/factor_base.h"
#include <limits>
#include <queue>

namespace srrg2_solver {

  
  struct VariableVisitEntry {
    VariableVisitEntry(VariableBase* variable_) {
      variable = variable_;
      cost     = std::numeric_limits<float>::max();
    }

    VariableVisitEntry(FactorBase* factor_, int parent_pos_, int var_pos_, float cost_);

    // called by visit algorithm when a variable
    // is put in the frontier
    void expand(FactorBase* factor_, int parent_pos_, int var_pos_, float cost_);

    FactorBase* factor     = 0;
    int parent_pos         = 0;
    int var_pos            = 0;
    float cost             = 0;
    VariableBase* variable = 0;
    VariableBase* parent   = 0;
    int num_visits         = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct VariableVisitEntryContainer {
    VariableVisitEntry* at(VariableBase::Id id);
    VariableVisitEntry* add(const VariableVisitEntry& entry);
    void clear();

    //protected:
    std::map<VariableBase::Id, VariableVisitEntry> _entries;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct FactorGraphVisitEntryQueueCompare {
    inline bool operator()(const VariableVisitEntry& a, const VariableVisitEntry& b) const {
      if (b.cost < a.cost)
        return true;
      if (b.cost > a.cost)
        return false;
      if (b.variable->graphId() > a.variable->graphId())
        return true;
      if (b.variable->graphId() < a.variable->graphId())
        return false;
      return b.factor->graphId()>a.factor->graphId();
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphVisitEntryQueue = std::priority_queue<VariableVisitEntry,
                                                         std::vector<VariableVisitEntry>,
                                                         FactorGraphVisitEntryQueueCompare>;

} // namespace srrg2_solver
