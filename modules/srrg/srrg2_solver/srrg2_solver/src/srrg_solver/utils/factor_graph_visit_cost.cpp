#include "factor_graph_visit_cost.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  

  float FactorGraphVisitCostUniform::cost(const FactorBase& factor,
                                          int var_from_pos,
                                          int var_to_pos){
    assert(var_from_pos>=0 && "variable<0" );
    assert(var_to_pos<factor.numVariables() && "variable>=num_variables" );
    return param_factor_cost.value();
  }

  float FactorGraphVisitCostCount::cost(const FactorBase& factor,
                                        int var_from_pos,
                                        int var_to_pos){
    assert(var_from_pos>=0 && "variable<0" );
    assert(var_to_pos<factor.numVariables() && "variable>=num_variables" );
    assert(param_touch_map.value() && "no touch map");
    std::map<VariableBase::Id, int>& touch_map = param_touch_map.value()->touch_map;
    auto var_id=factor.variableId(var_to_pos);
    
    auto it=touch_map.find(var_id);
    if (it==touch_map.end()) {
      touch_map[var_id]=100;
    }
    int& touch=touch_map[var_id];
    if (touch==0) {
      return param_factor_cost.value();
    }
    touch-=param_touch_discount.value();
    if (touch<=0) {
      touch=0;
      std::cerr << "unlock " << var_id << std::endl;
      return param_factor_cost.value();
    }
    return -1.f;
  }
}
