#pragma once
#include <srrg_data_structures/abstract_ptr_map.h>
#include <srrg_boss/serializer.h>
#include "factor_base.h"

namespace srrg2_solver {

  using namespace srrg2_core;

  template <typename PtrType_>
  struct IdPtrCompare_ {
    using PtrType = PtrType_;
    inline bool operator ()(const PtrType_& a, const PtrType_& b) const {
      return a->graphId()< b->graphId();
    }
  };

  /*!@brief  General interface that defines the basic functionality to manage a factor graph.
    In the derived class you must specify the accessors to the id to factors and id to variables
    containers.
   */

  class FactorGraphInterface {
  public:
    using FactorRawPtrSet         = std::set<FactorBase*, IdPtrCompare_<FactorBase*> >;
    using FactorRawPtrSetIterator = FactorRawPtrSet::iterator;
    using Id                        = int64_t;
    /*! Accessor for a variable using the graph id
      @param[in] id the graph id of the variable
      @return The variable pointer
     */
    VariableBase* variable(VariableBase::Id id);
    /*! Accessor for a factor using the graph id
      @param[in] id the graph id of the factor
      @return The factor pointer
     */
    FactorBase* factor(FactorBase::Id id);
    void printVariables();
    virtual ~FactorGraphInterface();
    /*! @return The container of variable pointers  */
    virtual IdVariablePtrContainer& variables() = 0;
    /*! @return The container of factor pointers  */
    virtual IdFactorPtrContainer& factors() = 0;

    virtual void clear();

    FactorRawPtrSet& factors(const VariableBase* v); // returns the factors of this variable
    
    virtual void addVariable(VariableBase* v) = 0;
    virtual void addFactor(FactorBase* f) = 0;
    virtual void removeVariable(VariableBase* v);
    virtual void removeFactor(FactorBase* f);

    void write(const std::string& filename);

  protected:
    /*! Connect a factor with the corresponding variables
      @return Number of variables correctly connected
     */
    virtual void bindFactor(FactorBase* factor);
    /*! Disconnect a factor from the corresponding variables
     */
    virtual void unbindFactor(FactorBase* factor);
    /*! Call bindFactor() for each factor in the interface
      @return Total number of variables connected
    */
    virtual void bindFactors();
    /*! Remove all the factors and variables from the interface */
    
    std::map<VariableBase::Id, FactorRawPtrSet> _var_to_factors;
    
    void _write(srrg2_core::Serializer& ser, std::set<FactorBase*>& factors);

    FactorRawPtrSet& _v2f(VariableBase::Id id, bool insert=false);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphInterfacePtr =
    std::shared_ptr<FactorGraphInterface>; /*!< Shared pointer
                                             to FactorGraphInterface */
}// end namespace

