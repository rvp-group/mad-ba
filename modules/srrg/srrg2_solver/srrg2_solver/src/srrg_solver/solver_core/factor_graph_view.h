#pragma once
#include "factor_graph_interface.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  /*! @brief View of a factor graph, does not have ownership of variables and factors.
   As such the accessors to the id to variables and id to factors container have a raw pointer
   as value. The compatibility among different container types (same key type different value type)
   is achieved through AbstractMap_, see srrg2_core.
   */
  class FactorGraphView : public FactorGraphInterface {
  public:
    using IdVariableRawPtrMap = AbstractMap_<VariableBase::Id, VariableBase*>;
    using IdFactorRawPtrMap   = AbstractMap_<FactorBase::Id, FactorBase*>;
    virtual ~FactorGraphView();
    /*! @return The container of variable pointers*/
    IdVariablePtrContainer& variables() override;
    /*! @return The container of factor pointers*/
    IdFactorPtrContainer& factors() override;

    void addVariable(VariableBase* v) override;
    void addFactor(FactorBase* f) override;
    void removeVariable(VariableBase* v) override;
    void removeFactor(FactorBase* f) override;

    // makes the union between this view and another one
    void add(FactorGraphInterface& src, int level=-1);
    // makes the union between this view and another one,
    // selecting the factors with proper id from src.
    
    void addFactors(FactorGraphInterface& src, const std::set<FactorBase::Id>& factors);

    // makes the union between this view and another one, selecting the variables in the src,
    // and all factors connected, that join variables in the set
    void addVariables(FactorGraphInterface& src, const std::set<VariableBase::Id>&, int level=-1);
    
  protected:
    IdVariableRawPtrMap _variables; /*!< Id to variable raw pointer container */
    IdFactorRawPtrMap _factors;     /*!< Id to factor raw pointer container */

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphViewPtr = std::shared_ptr<FactorGraphView>; /*!<Shared pointer
                                                                 to FactorGraphView */

}// end namespace
