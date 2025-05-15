#include "factor_graph_interface.h"
#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
// included here since I instantiate all methods
namespace srrg2_solver {

  using namespace std;
  using namespace srrg2_core;

  FactorGraphInterface::~FactorGraphInterface() {
  }

  void FactorGraphInterface::clear() {
    variables().clear();
    factors().clear();
    _var_to_factors.clear();
  }

  FactorGraphInterface::FactorRawPtrSet& FactorGraphInterface::_v2f(VariableBase::Id id,
                                                                      bool insert) {
    auto it = _var_to_factors.find(id);
    if (it != _var_to_factors.end()) {
      return it->second;
    } else {
      if (!insert)
        std::runtime_error("FactorGraphInrerface::_v2f, no var in");
    }
    auto ret = _var_to_factors.insert(std::make_pair(id, FactorRawPtrSet()));
    return ret.first->second;
  }

  void FactorGraphInterface::_write(srrg2_core::Serializer& ser,
                                    std::set<FactorBase*>& selected_factors) {
    cerr << endl;
    std::set<VariableBase*> selected_variables;
    for (auto f : selected_factors) {
      if (!factor(f->graphId())) {
        throw std::runtime_error("no factor in serialization");
      }
      int num_var = f->numVariables();
      for (int v_idx = 0; v_idx < num_var; ++v_idx) {
        VariableBase* v = f->variable(v_idx);
        if (v)
          selected_variables.insert(v);
      }
    }
    int object_count = 0;
    for (VariableBase* v : selected_variables) {
      ++object_count;
      if (!(object_count % 1000))
        cerr << "\robjects written" << object_count;
      ser.writeObject(*v);
    }
    for (FactorBase* f : selected_factors) {
      ++object_count;
      if (!(object_count % 1000))
        cerr << "\robjects written" << object_count;
      ser.writeObject(*f);
    }
  }

  void FactorGraphInterface::write(const std::string& filename) {
    Serializer ser;
    ser.setFilePath(filename);
    cerr << "writing view on file [ " << filename << "]" << endl;
    std::set<FactorBase*> selected_factors;
    for (auto f_it : factors()) {
      selected_factors.insert(f_it.second);
    }
    _write(ser, selected_factors);
    cerr << "done" << endl;
  }

  void FactorGraphInterface::printVariables() {
    for (auto it = variables().begin(); it != variables().end(); ++it) {
      const VariableBase* const v = it.value();
      cerr << "id: " << v->graphId() << " " << v->status() << endl;
    }
  }

  VariableBase* FactorGraphInterface::variable(VariableBase::Id id) {
    auto it = variables().find(id);
    if (it == variables().end()) {
      return 0;
    }
    VariableBase* v = it.value();
    return v;
  }

  FactorBase* FactorGraphInterface::factor(FactorBase::Id id) {
    auto it = factors().find(id);
    if (it == factors().end()) {
      return 0;
    }
    FactorBase* f = it.value();
    return f;
  }

  void FactorGraphInterface::bindFactor(FactorBase* f) {
    f->bind(variables());
    for (int pos = 0; pos < f->numVariables(); ++pos) {
      _v2f(f->variableId(pos)).insert(f);
    }
  }

  void FactorGraphInterface::unbindFactor(FactorBase* f) {
    // cerr << __PRETTY_FUNCTION__ << "| " << this << endl;
    for (int var_idx = 0; var_idx < f->numVariables(); ++var_idx) {
      VariableBase* v = f->variable(var_idx);
      if (v) {
        _v2f(v->graphId()).erase(f);
      }
      f->setVariable(var_idx, 0);
    }
  }

  void FactorGraphInterface::bindFactors() {
    //    std::cerr << "BindFactors " << this << std::endl;
    _var_to_factors.clear();
    for (auto f_it: factors()) {
      bindFactor(f_it.second);
    }
  }

  FactorGraphInterface::FactorRawPtrSet& FactorGraphInterface::factors(const VariableBase* v) {
    return _v2f(v->graphId());
  }

  void FactorGraphInterface::removeFactor(FactorBase* f) {
    // cerr << __PRETTY_FUNCTION__ << "| " << this << " " << f << endl;
    auto it = factors().find(f->graphId());
    if (it == factors().end()) {
      throw std::runtime_error("removeFactor| no factor");
    }
    if (f != it.value())
        throw std::runtime_error("factor ptr mismatch");
    unbindFactor(f);
    // cerr << "fac_remove, erasing factor" << f << endl;
  }

  void FactorGraphInterface::removeVariable(VariableBase* v) {
    if (!_v2f(v->graphId()).empty()) {
      for (auto f: _var_to_factors[v->graphId()]) {
        cerr << f->graphId() << endl;
      }
      throw std::runtime_error ("Cannot remove a variable still bound to factors");
    }
    auto it = variables().find(v->graphId());
    if (it == variables().end()) {
      throw std::runtime_error("var not in view");
    }
    assert(v == it.value() && "variables ptr mismatch");
    // cerr << "Var id: "<< v->graphId()<< endl;
    _var_to_factors.erase(v->graphId());
  }

} // namespace srrg2_solver
