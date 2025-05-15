#pragma once
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_boss/deserializer.h"

namespace srrg2_solver {
  struct EndEpoch: public Serializable {
    void serialize(ObjectData& odata,  IdContext& context) override;
    void deserialize(ObjectData& odata,  IdContext& context) override;
    int epoch=0;
  };
  
  using EndEpochPtr=std::shared_ptr<EndEpoch>;
  
  class FactorGraphIncrementalHandlerBase {
  public:
    inline void setGraph(FactorGraphPtr graph_) { _graph=graph_; }
    inline void setPathType(const std::string& variable_path_type_) { _variable_path_type=variable_path_type_; }
  protected:
    VariableBase::Id order(Serializable* s);  
    void print(Serializable* s);
    FactorGraphPtr _graph;
    std::string _variable_path_type="VariableSE3QuaternionRightAD";
  };
    
  class FactorGraphIncrementalSorter: public FactorGraphIncrementalHandlerBase {
  public:
    void compute();
    void print();
    std::vector<Serializable*>& items() {return _items;}
  protected:
    std::vector<Serializable*> _items;
  };


  class FactorGraphIncrementalReader: public FactorGraphIncrementalHandlerBase {
  public:
    inline void setRelaxed(std::set<VariableBase::Id>& relaxed_) {_relaxed=relaxed_;}

    void setFilePath(const std::string& filename);
    bool readEpoch(std::set<VariableBase::Id>& new_vars,
                   std::set<FactorBase::Id>& new_factors);
    bool readEpochTag(std::set<VariableBase::Id>& new_vars,
                      std::set<FactorBase::Id>& new_factors);
    std::set<VariableBase::Id>& relaxed() {return _relaxed;}
  protected:
    std::set<VariableBase::Id> _added_ids;
    void flushPending(std::set<VariableBase::Id>& new_vars,
                      std::set<FactorBase::Id>& new_factors);
    bool canAdd(FactorBasePtr f);
    Deserializer _des;
    VariableBase::Id _current_epoch = -1;
    std::list<VariableBasePtr> _pending_vars;
    std::list<FactorBasePtr> _pending_factors;
    std::set<VariableBase::Id> _relaxed;
  };
}
