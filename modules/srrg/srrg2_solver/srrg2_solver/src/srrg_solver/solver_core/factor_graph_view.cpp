#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
#include "factor_graph_view.h"
// included here since I instantiate all methods
namespace srrg2_solver {

  using namespace std;
  using namespace srrg2_core;

  FactorGraphView::~FactorGraphView() {
    clear();
  }

  IdVariablePtrContainer& FactorGraphView::variables() {
    return _variables;
  }

  IdFactorPtrContainer& FactorGraphView::factors() {
    return _factors;
  }

  void FactorGraphView::addVariable(VariableBase* v) {
    auto v_other=variable(v->graphId());
    if (v_other){
      if (v_other==v)
        return;
      throw std::runtime_error ("addVariable, another different with same id exists");
    }
    _variables.insert(std::make_pair(v->graphId(), v));
    _v2f(v->graphId(), true);
  }
  
  void FactorGraphView::addFactor(FactorBase* f) {
    FactorBase* f_other=factor(f->graphId());
    if(f_other) {
      if (f_other==f) {
        bindFactor(f);
        return;
      }
      throw std::runtime_error ("addFactor, another factor with same id exists");
    }
    _factors.insert(std::make_pair(f->graphId(), f));
    bindFactor(f);
  }

  void FactorGraphView::removeVariable(VariableBase* v) {
    //cerr << __PRETTY_FUNCTION__ << "| " << this << " " << v << endl;
    FactorGraphInterface::removeVariable(v);
    auto it=_variables.find(v->graphId());
    _variables.erase(it);
  }

  void FactorGraphView::removeFactor(FactorBase* f) {
    FactorGraphInterface::removeFactor(f);
    auto it=_factors.find(f->graphId());
    _factors.erase(it);
  }

  // makes the union between this view and another one
  void FactorGraphView::add(FactorGraphInterface& src, int level) {
    std::set<FactorBase::Id> factor_ids;
    for (auto f_it: src.factors()) {
      FactorBase* f=f_it.second;
      if (level!=-1 && f->level()!= level)
        continue;
      factor_ids.insert(f->graphId());
    }
    addFactors(src, factor_ids);
  }
   
  // makes the union between this view and another one,
  // selecting the factors with proper id from src.
    
  void FactorGraphView::addFactors(FactorGraphInterface& src,
                                   const std::set<FactorBase::Id>& factor_ids) {

    // we add to the factors all factors in vectors;
    // we add to the variables all variables connected by the factors
    std::set<VariableBase*> added_vars;
    std::set<FactorBase*>   added_factors;
    
    // we add to the variables of the view all variables in vector
    for (auto graph_id : factor_ids) {
      FactorBase* f = src.factor(graph_id);
      if (!f) {
        throw std::runtime_error("factor not in graph");
      }
      added_factors.insert(f);
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* v = src.variable(f->variableId(pos));
        if (!v) {
          throw std::runtime_error("no v in src");
        }
        added_vars.insert(v);
      }
    }
    for (auto v: added_vars) {
      addVariable(v);
    }
    for (auto f: added_factors){
      addFactor(f);
    }

  }

  // makes the union between this view and another one, selecting the variables in the src,
  // and all factors connected, that join variables in the set
  void FactorGraphView::addVariables(FactorGraphInterface& src,
                                     const std::set<VariableBase::Id>& variable_ids,
                                     int level){
    // we add to the variables of the view all variables in vector
    FactorGraphInterface::FactorRawPtrSet candidate_factors;
    for (auto graph_id : variable_ids) {
      VariableBase* v = src.variable(graph_id);
      if (!v) {
        throw std::runtime_error("variable not in graph");
      }
      addVariable(v);
      candidate_factors.insert(src.factors(v).begin(), src.factors(v).end());
    }
    
    for(auto f: candidate_factors) {
      if (level!=-1 && f->level()!=level)
        continue;
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        auto id=f->variableId(pos);
        if (! variable_ids.count(id)) {
          f=0;
          break;
        }
      }
      if (f) {
        addFactor(f);
      }
    }
  }

}
