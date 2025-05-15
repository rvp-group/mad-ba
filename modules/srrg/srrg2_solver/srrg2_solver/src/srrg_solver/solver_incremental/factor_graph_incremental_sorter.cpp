#include "factor_graph_incremental_sorter.h"
#include <iostream>

namespace srrg2_solver {
  using namespace std;

  void EndEpoch::serialize(ObjectData& odata,  IdContext& context){
    odata.setInt("epoch", epoch);
  }
  void EndEpoch::deserialize(ObjectData& odata,  IdContext& context){
    epoch=odata.getInt("epoch");
  }

  void FactorGraphIncrementalSorter::compute() {
    _items.clear();
      
    for (auto v_it: _graph->variables()) {
      _items.push_back(v_it.second);
    }
    for (auto f_it: _graph->factors()) {
      _items.push_back(f_it.second);
    }

    std::sort(_items.begin(),
              _items.end(),
              [this](Serializable* s1, Serializable* s2)->bool {
                auto o1=order(s1);
                auto o2=order(s2);
                if (o1<o2)
                  return true;
                if (o1>o2)
                  return false;
                VariableBase* v1=dynamic_cast<VariableBase*>(s1);
                FactorBase* f1=dynamic_cast<FactorBase*>(s1);
                VariableBase* v2=dynamic_cast<VariableBase*>(s2);
                FactorBase* f2=dynamic_cast<FactorBase*>(s2);
                if ( (f1 && f2) || (v1 && v2))
                  return false;
                if (v1 && f2)
                  return true;
                return false;
              }
              );
  }

  void FactorGraphIncrementalHandlerBase::print(Serializable* s) {
      cerr << s->className();
      VariableBase* v=dynamic_cast<VariableBase*>(s);
      if (v)
        cerr << "id: " << v->graphId() << endl;
      FactorBase* f=dynamic_cast<FactorBase*>(s);
      if (f) {
        cerr << "id: " << f->graphId() << "[ ";
        for (int i=0; i<f->numVariables(); ++i){
          cerr << f->variableId(i) << " ";
        }
        cerr << "]" << endl;
      }    
    }

  void FactorGraphIncrementalSorter::print() {
    VariableBase::Id old_order=-1;
    for (auto i:_items) {
      VariableBase::Id o=order(i);
      if (o>old_order) {
        cerr << "NEW EPOCH: " << o << endl;
        old_order=o;
      }
      FactorGraphIncrementalHandlerBase::print(i);
    }
  }
  
  VariableBase::Id FactorGraphIncrementalHandlerBase::order(Serializable* s) {
    VariableBase* v=dynamic_cast<VariableBase*>(s);
    if (v) {
      if (v->status()==VariableBase::Fixed)
        return -1;
      if (v->className()==_variable_path_type)
        return v->graphId();
      //look for all factors in the variable and return as order the
      //minimum order of the factor
      VariableBase::Id min_order=std::numeric_limits<VariableBase::Id>::max();
      for (auto f: _graph->FactorGraphInterface::factors(v)) {
        min_order=std::min(min_order,order(f));
      }
      return min_order;
    }
    FactorBase* f=dynamic_cast<FactorBase*>(s);
    if (f) {
      VariableBase::Id max_order=0;
      for (int i=0; i<f->numVariables(); ++i) {
        VariableBase* other_v=f->variable(i);
        if (other_v->status()==VariableBase::Fixed)
          continue;
        if (other_v->className()==_variable_path_type)
          max_order=std::max(max_order,other_v->graphId());
      }
      return max_order;
    }
    return -1;
  }


  void FactorGraphIncrementalReader::setFilePath(const std::string& filename) {
    _des.setFilePath(filename);
    _pending_vars.clear();
    _pending_factors.clear();
    _current_epoch = -1;
  }

  bool FactorGraphIncrementalReader::canAdd(FactorBasePtr f) {
    for (int i=0; i<f->numVariables(); ++i) {
      if (! _added_ids.count(f->variableId(i)) )
          return false;
    }
    return true;
  }

  bool FactorGraphIncrementalReader::readEpoch(std::set<VariableBase::Id>& new_vars,
                                               std::set<FactorBase::Id>& new_factors) {
    SerializablePtr o;
    while( (o=_des.readObjectShared()) ) {
      if (! o ) {
        if (_pending_vars.empty() && _pending_factors.empty())
          return false;
        flushPending(new_vars, new_factors);
        return true;
      }
      FactorBasePtr f=dynamic_pointer_cast<FactorBase>(o);
      VariableBasePtr v=dynamic_pointer_cast<VariableBase>(o);
      if (v && _relaxed.count(v->graphId())) {
        v->setStatus(VariableBase::Active);
      }
      VariableBase::Id o_order=_current_epoch;
      if (v && v->className()==_variable_path_type) {
        o_order=order(o.get());
      }
      
      bool end_epoch=false;
      if (o_order > _current_epoch) {
        flushPending(new_vars, new_factors);
        _current_epoch=o_order;
        end_epoch=true;
      }
      if (f) _pending_factors.push_back(f);
      if (v) _pending_vars.push_back(v);
      
      if (end_epoch) return true;
      if (o_order >=0 && o_order <  _current_epoch) {
        throw std::runtime_error("graph is not sorted on this, please run the sorter first order:"+
                                 to_string(o_order) + " epoch: " + to_string(_current_epoch));
      }
    }
    return false;
  }

  bool FactorGraphIncrementalReader::readEpochTag(std::set<VariableBase::Id>& new_vars,
                                                  std::set<FactorBase::Id>& new_factors) {
    SerializablePtr o;
    _current_epoch=-1;
    cerr << __PRETTY_FUNCTION__ << endl;
    while( (o=_des.readObjectShared()) ) {
      if (! o ) {
        if (_pending_vars.empty() && _pending_factors.empty())
          return false;
        flushPending(new_vars, new_factors);
        return true;
      }
      EndEpochPtr end_epoch=dynamic_pointer_cast<EndEpoch>(o);
      if(end_epoch) {
        cerr << "epoch over, new epoch: " << end_epoch->epoch << endl;
        flushPending(new_vars, new_factors);
      }
      FactorBasePtr f=dynamic_pointer_cast<FactorBase>(o);
      VariableBasePtr v=dynamic_pointer_cast<VariableBase>(o);
      if (v && _relaxed.count(v->graphId())) {
        v->setStatus(VariableBase::Active);
      }
      if(end_epoch)
        _current_epoch=end_epoch->epoch;
      if (f) _pending_factors.push_back(f);
      if (v) _pending_vars.push_back(v);
      if (end_epoch) return true;
    }
    return false;
  }

  void FactorGraphIncrementalReader::flushPending(std::set<VariableBase::Id>& new_vars,
                                                  std::set<FactorBase::Id>& new_factors){
    new_vars.clear();
    new_factors.clear();
    for (VariableBasePtr& v: _pending_vars) {
      _graph->addVariable(v);
      _added_ids.insert(v->graphId());
      new_vars.insert(v->graphId());
    }
    _pending_vars.clear();
    for (auto f_it=_pending_factors.begin(); f_it!=_pending_factors.end();) {
      FactorBasePtr& f=*f_it;
      auto erased=f_it;
      ++f_it;
      if (canAdd(f)) {
        _graph->addFactor(f);
        new_factors.insert(f->graphId());
        _pending_factors.erase(erased);
      }
    }
  }

}
