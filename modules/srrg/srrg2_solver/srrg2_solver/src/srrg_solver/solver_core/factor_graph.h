#pragma once
#include "factor_graph_interface.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Materialization of a factor graph. A factor graph *owns* its variables and factors, so
    in this case the id to variable and id to factors container have as value a shared pointer.

    An example of factor graph interface without ownership is FactorGraphView.
  */
  class FactorGraph : public FactorGraphInterface, public Serializable {
  public:
    /*! Add a variable to the factor graph
      @param[in] var a shared pointer to variable
    */
    void addVariable(VariableBasePtr var);
    /*! Add a factor to the graph
      @param[in] factor a shared pointer to factor
    */
    void addFactor(FactorBasePtr factor);
    /*! Remove factor from the graph
      @param[in] factor a shared pointer to factor
    */
    void removeFactor(FactorBasePtr factor);
    /*! Auxiliary function used to remove a factor from the graph using a raw pointer
      @param[in] factor raw pointer to factor
    */
    void removeFactor(FactorBase* factor);

    void removeVariable(VariableBasePtr var);

    void removeVariable(VariableBase* var);

    virtual ~FactorGraph();

    IdVariablePtrContainer& variables() override;
    IdFactorPtrContainer& factors() override;

    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;


    /*! Read a graph from a file (static method)
      @param[in] filename
      @return shared pointer to the factor graph loaded
    */
    static std::shared_ptr<FactorGraph> read(const std::string& filename);
    /*! @return Last graph id*/
    const Id& lastGraphId() const {
      return _last_graph_id;
    }

    /*! Write a factor graph on a file
      @param[in] filename
    */
    void write(const std::string& filename);

    VariableBasePtr detachVariable(VariableBase* v);
    FactorBasePtr detachFactor(FactorBase* f);

    void setSerializationLevel(int level_) {_level_serialization=level_;}
  protected:
    using IdVariablePtrMap =
      AbstractPtrMap_<VariableBase::Id, VariableBase, std::shared_ptr<VariableBase>>;
    using IdFactorPtrMap = AbstractPtrMap_<FactorBase::Id, FactorBase, std::shared_ptr<FactorBase>>;
    IdFactorPtrMap _factors;     /*!< Id to factor shared pointer container */
    IdVariablePtrMap _variables; /*!< Id to variable shared pointer container */

    void addVariable(VariableBase* v) override;
    void addFactor(FactorBase* f) override;

  private:
    Id _last_graph_id = 0; /*!< Last graph id */
    int _level_serialization = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphPtr = std::shared_ptr<FactorGraph>; /*!< Shared pointer to FactorGraph */
} // namespace srrg2_solver
