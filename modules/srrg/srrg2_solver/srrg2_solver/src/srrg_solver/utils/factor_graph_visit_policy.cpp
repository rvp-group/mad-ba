#include "factor_graph_visit_policy.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  bool FactorGraphVisitPolicyBase::match(const FactorBase& factor,
                                         int var_from_pos_,
                                         int var_to_pos_) {
    if (param_var_from_pos.value() > -1 && param_var_from_pos.value() != var_from_pos_) {
      //      std::cerr << "no match (from) " <<  param_var_from_pos.value() << "!=" << var_from_pos_ << std::endl;
      return false;
    }
    if (param_var_to_pos.value() > -1 && param_var_to_pos.value() != var_to_pos_) {
      //std::cerr << "no match (to) " <<  param_var_to_pos.value() << "!=" << var_to_pos_ << std::endl;
      return false;
    }
    return true;
  }

  bool FactorGraphVisitPolicyByType::match(const FactorBase& factor,
                                           int var_from_pos_,
                                           int var_to_pos_) {
    if (!FactorGraphVisitPolicyBase::match(factor, var_from_pos_, var_to_pos_))
      return false;
    if (factor.className() != param_factor_classname.value()) {
      //std::cerr << "no match (type) " <<  factor.className() << "!=" << param_factor_classname.value() << std::endl;
      
      return false;
    }
    //std::cerr << "match! " << factor.className() << " " << var_from_pos_ << " " << var_to_pos_ << std::endl;
    return true;
  }
} // namespace srrg2_solver
