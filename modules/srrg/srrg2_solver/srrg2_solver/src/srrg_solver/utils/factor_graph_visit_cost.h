#pragma once
#include "factor_graph_visit_entry.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include <srrg_config/property_configurable_vector.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  // cost function for traversing a factor and going from a parent to a child
  // variable
  class FactorGraphVisitCostBase : public Configurable {
  public:
    using BaseType = Configurable;
    using ThisType = FactorGraphVisitCostBase;

    virtual float
    cost(const FactorBase& factor, int var_from_pos, int var_to_pos) = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  

  class FactorGraphVisitCostUniform : public FactorGraphVisitCostBase {
  public:
    PARAM(PropertyFloat, factor_cost, "cost of traversing the factor", 1.0, 0);

    float cost(const FactorBase& factor,
               int var_from_pos,
               int var_to_pos) override;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
    
  struct FactorGraphVisitTouchMap : public Configurable {
    std::map<VariableBase::Id, int> touch_map;
  };

  // cost function for traversing a factor and going from a parent to a child
  // variable
  // it keeps a list of "activation costs" for the variables
  // each visit discounts the cost of a certain factor, and when the cost is 0
  // the variable is unlocked and can be visited
  class FactorGraphVisitCostCount : public FactorGraphVisitCostUniform {
  public:
    using BaseType = Configurable;
    using ThisType = FactorGraphVisitCostBase;
    PARAM(PropertyInt, touch_discount, "cost of touching the var", 1, 0);
    PARAM(PropertyConfigurable_<FactorGraphVisitTouchMap>,
          touch_map,
          "map to hold the unlocks for the variables",
          0,
          0);
    
    virtual float
    cost(const FactorBase& factor, int var_from_pos, int var_to_pos);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_solver
