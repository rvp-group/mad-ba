#include "solver_incremental.h"
#include <unistd.h>

namespace srrg2_solver{
  using namespace srrg2_core;
  using namespace std;
  
  VariableBase* SolverIncremental::findGauge(FactorGraphInterface& view) {
    for (auto v_it: view.variables()) {
      VariableBase* v=v_it.second;
      if (v->className()!=param_gauge_type.value())
        continue;
      if (_parameter_ids.count(v->graphId()))
        continue;
      return v;
    }
    return nullptr;
  }

  void SolverIncremental::activateVariables() {
    for (auto v_it: _graph->variables()) {
      VariableBase* v=v_it.second;
      if (v->status()==VariableBase::NonActive)
        v->setStatus(VariableBase::Active);
    }
  }

  int SolverIncremental::suppressNonInitialized() {
    int num_init=0;
    for (auto v_it: _graph->variables()) {
      VariableBase* v=v_it.second;
      if (v->status()==VariableBase::Fixed)
        continue;
      if (_initializer.isInit(v)) { 
        ++num_init;
      } else {
        v->setStatus(VariableBase::NonActive);
      }
    }
    return num_init;
  }

    // call once before the machine runs
  void SolverIncremental::setGraph(FactorGraphPtr graph_){
    _global_gauge=0;
    _graph=graph_;
    param_global_solver.value()->setGraph(_graph);
    _initializer.setGraph(*_graph);
  }

  // call once, passing the factor pool that is new since the last epoch
  VariableBase::Id SolverIncremental::compute(const IdSet& factor_ids, bool force) {
    Chrono time_total("TOTAL| time:", &_chrono_map, true);
    _pending_view.addFactors(*_graph, factor_ids);
    if (! factor_ids.size())
      return 0;

    for (auto v_it: _pending_view.variables()) {
      if (v_it.second->status()==VariableBase::Fixed)
        _parameter_ids.insert(v_it.first);
    }
    
    _initializer.parameterIds()=_parameter_ids;

    _local_gauge=findGauge(_pending_view);
    if (! _global_gauge) {
      _global_gauge=_local_gauge;
    }
    
    if (! _global_gauge)
      return -1;
    ++_elapsed_global_steps;
    // std::cerr << "_global_gauge: " << _global_gauge->graphId() << std::endl;
    // std::cerr << "_local_gauge: " << _local_gauge->graphId() << std::endl;
    // std::cerr << "_pending factors: " << _pending_view.factors().size() << endl;
    // std::cerr << "_pending vars: " << _pending_view.variables().size() << endl;
    _local_gauge->setStatus(VariableBase::Fixed);
    activateVariables();
    int num_init = 0;
    {
      Chrono time_init("INITV| time:", &_chrono_map, true);
      _initializer.updateGraph();
      _initializer.compute();
      num_init=suppressNonInitialized();
      _local_gauge->setStatus(VariableBase::Active);
    }

    //cerr <<  "num_init: " << num_init << endl;
    if (! num_init)
      return _local_gauge->graphId();
    if (_elapsed_global_steps<param_global_step.value()) {
      Chrono time_init("LOCAL| time:", &_chrono_map, true);
      //cerr << "local opt: " << endl;

      // lock all variables that are not in the pending view,
      // and optimize a portion, leaving those fixed
      IdSet pending_variables;
      for (auto v_it: _pending_view.variables())
        pending_variables.insert(v_it.first);
      FactorGraphView outer_view;
      outer_view.addVariables(*_graph, pending_variables);

      IdSet fixed_variables;
      for (auto v_it: outer_view.variables()) {
        auto v_id=v_it.first;
        if (!_parameter_ids.count(v_id) && !_pending_view.variable(v_id))
          fixed_variables.insert(v_id);
      }
      
      
      _local_gauge->setStatus(VariableBase::Fixed);
      for (auto v_id: fixed_variables) {
        VariableBase* v=_graph->variable(v_id);
        v->setStatus(VariableBase::Fixed);
      }

      param_local_solver.value()->setGraph(outer_view);
      param_local_solver.value()->compute();
      for (auto v_id: fixed_variables) {
        VariableBase* v=_graph->variable(v_id);
        v->setStatus(VariableBase::Active);
      }
      _local_gauge->setStatus(VariableBase::Active);
      return _local_gauge->graphId();
    }
    
    //cerr << "global opt: " << _graph->variables().size() << " " << _graph->factors().size() << endl;
    {
      Chrono time_init("GLOBAL| time:", &_chrono_map, true);
      _global_gauge->setStatus(VariableBase::Fixed);
      param_global_solver.value()->setGraph(_graph);
      param_global_solver.value()->compute();
      _global_gauge->setStatus(VariableBase::Active);
      _elapsed_global_steps=0;
      _pending_view.clear();
    }
    return _local_gauge->graphId();
  }

  // call at the end
  void SolverIncremental::finalize(){
    Chrono finalize("FINALIZE| time:", &_chrono_map, true);
    if (! _global_gauge)
      return;
    _finalized_factors.clear();
    for (auto f_it: _graph->factors()) {
      _finalized_factors.insert(f_it.first);
    }
    _global_gauge->setStatus(VariableBase::Fixed);
    param_global_solver.value()->setGraph(_graph);
    param_global_solver.value()->compute();
    _global_gauge->setStatus(VariableBase::Active);
    std::cerr << "finalize done" << endl;
  }

  const IdSet& SolverIncremental::finalizedFactors() const  {
    return _finalized_factors;
  }

  void SolverIncremental::computeSurroundingView(FactorGraphView& view)  {
    view.clear();
    if (! _global_gauge)
      return;
    IdSet visited;
    
    FactorGraphVisit& visit=*param_surrounding_visit.value();
    visit.tainted()=_parameter_ids;
    visit.setGraph(*_graph);
    visit.sources().clear();
    visit.sources().insert(_local_gauge->graphId());
    visited.insert(_parameter_ids.begin(), _parameter_ids.end());
    visited.insert(_local_gauge->graphId());
    visit.compute();
    for (auto v_it:_graph->variables()){
      auto v_id=v_it.first;
      auto entry=visit.entries().at(v_id);
      if (! entry)
        return;
      if (entry->cost >=0 && entry->cost<visit.param_max_cost.value() ) {
        visited.insert(v_id);
      } 
    }
    view.addVariables(*_graph, visited);
    //cerr << "surrounding size: " << view.variables().size() << " " << view.factors().size() << endl;
  }

}
