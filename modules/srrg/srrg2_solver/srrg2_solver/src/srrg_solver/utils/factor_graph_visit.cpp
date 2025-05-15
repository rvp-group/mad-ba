#include "factor_graph_visit.h"
#include <algorithm>

using namespace std;

namespace srrg2_solver {
  using namespace srrg2_core;

  float FactorGraphVisit::cost(FactorBase& factor, int parent_pos, int var_pos) {
    for (size_t i = 0; i < param_cost_policies.size(); ++i) {
      FactorGraphVisitPolicyBase* policy = param_cost_policies[i].get();
      float c                            = policy->cost(factor, parent_pos, var_pos);
      if (c > 0) {
        return c;
      }
    }
    return -1;
  }

  void FactorGraphVisit::compute() {
    if (!_graph) {
      throw std::runtime_error("FactorGraphVisit::compute|graph not set");
    }

    if (_sources.empty()) {
      throw std::runtime_error("FactorGraphVisit::compute|sources not set");
    }

    _entries.clear();
    IdVariablePtrContainer& variables = _graph->variables();
    for (IdVariablePtrContainer::iterator it = variables.begin(); it != variables.end(); ++it) {
      VariableBase* v = it.value();
      _entries.add(VariableVisitEntry(v));
    }
    _queue = FactorGraphVisitEntryQueue();

    for (auto s: _sources) {
      VariableVisitEntry* e = _entries.at(s);
      assert(e && "source not in graph");
      e->cost = 0;
      _queue.push(*e);
    }

    while (!_queue.empty()) {
      VariableVisitEntry e = _queue.top();
      _queue.pop();
      VariableBase* v = e.variable;
      
      if (_tainted.count(v->graphId()))
        continue;
      
      //std::cerr << std::endl;
      //std::cerr << std::endl;
      //std::cerr << "EXPANDING: " << v->graphId() << std::endl;
      float v_cost = e.cost;

      VariableVisitEntry* stored_entry=_entries.at(v->graphId());
      if (stored_entry->cost < v_cost)
        continue;
      // scan all factors
      for (auto f : _graph->factors(v)) {
        int v_pos          = -1;
        if (!f->enabled())
          continue;

        // find index of var in factor
        for (int i = 0; i < f->numVariables(); ++i) {
          if (f->variable(i) == v) {
            e.var_pos = i;
            v_pos      = i;
            break;
          }
        }

        //std::cerr << "\tf: " << f << std::endl;
        if (!e.factor) {
          e.factor = f;
          assert(v == f->variable(e.var_pos) && "bookkeping error 1"); // ia dunno if it's this
        }

        // scan all variables
        for (int nv_pos = 0; nv_pos < f->numVariables(); ++nv_pos) {
          VariableBase* nv = f->variable(nv_pos);
          if (nv == v)
            continue;

          if (_tainted.count(nv->graphId()) )
            continue;

          VariableVisitEntry* ne = _entries.at(nv->graphId());
          assert(ne && "bookkeeping error 2");
          float d_cost = cost(*f, v_pos, nv_pos);
          //std::cerr << "\t\tnv=" << nv->graphId() << std::endl;
          //std::cerr << "\t\tv_pos=" << v_pos << std::endl;
          //std::cerr << "\t\tnv_pos=" << nv_pos << std::endl;
          //std::cerr << "\t\td_cost=" << d_cost << std::endl;
          //std::cerr << "\t\tv_cost=" << v_cost << std::endl;
          //std::cerr << "\t\tne_cost=" << ne->cost << std::endl;
          if (d_cost < 0) {
            continue;
          }
          ++ne->num_visits;
          if (v_cost + d_cost < ne->cost) {
            if (param_max_cost.value()>0.f && v_cost + d_cost > param_max_cost.value())
              continue;
            //std::cerr << "\t\tcost: " << d_cost << std::endl;
            ne->expand(f, v_pos, nv_pos, v_cost + d_cost);
            _queue.push(*ne);
          }
        }
      }
    }
  }

  float FactorGraphVisit::averageCost()  {
    int num_entries=0;
    double total_cost=0;
    for (auto  it: _graph->variables()) {
      const VariableVisitEntry* entry=_entries.at(it.second->graphId());
      if (! entry)
        continue;
      if (entry->num_visits) {
        ++num_entries;
        total_cost+=entry->cost;
      }
    }
    if (!num_entries)
      return 0.f;
    
    return total_cost/num_entries;
  }

  float FactorGraphVisit::maxCost()  {
    int num_entries=0;
    float max_cost=0;
    for (auto  it: _graph->variables()) {
      const VariableVisitEntry* entry=_entries.at(it.second->graphId());
      if (! entry)
        continue;
      if (entry->num_visits) {
        ++num_entries;
        max_cost=std::max(entry->cost, max_cost);
      }
    }
    if (!num_entries)
      return 0.f;
    return max_cost;
  }

} // namespace srrg2_solver
