#pragma once
#include "ad_variable.h"
#include "constraint_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  /*! @brief Auto diff Constraint factor class. It is defined through the same templates as
     ConstraintFactor_. In this case you have to specify in the derived class just the () operator,
     which takes as an argument a VariableTupleType. In the operator should compute just the
     Constraint vector. Remember that all the computation has to be performed in DualValuef.
  */
  template <int ConstraintDim_, typename... VariableTypes_>
  class ADConstraintFactor_ : public ConstraintFactor_<ConstraintDim_, VariableTypes_...> {
  public:
    using ThisType = ADConstraintFactor_<ConstraintDim_, VariableTypes_...>;
    using BaseType = ConstraintFactor_<ConstraintDim_, VariableTypes_...>;
    using VariableTupleType =
      typename BaseType::VariableTupleType; /*!< Extract the variable tuple type */
    static constexpr int NumVariables = BaseType::NumVariables;   /*!< Determine the
                                                                    number of variables*/
    static constexpr int ConstraintDim = BaseType::ConstraintDim; /*!<Constraint dimensionality*/
    using ADConstraintVectorType =
      Eigen::Matrix<DualValuef, ConstraintDim, 1>; /*!Constraint Vector type*/

    template <typename ThisType_, int idx>
    friend struct JUpdater;

    virtual ~ADConstraintFactor_() {
    }
    /*! Compute the Constraint vector in DualValuef
      @param[in] vars container of the variables involved in the factor
      @return The Constraint vector
    */
    virtual ADConstraintVectorType operator()(VariableTupleType& vars) = 0;

    /*! Compute Constraint vector (call operator()) and determine the Jacobian matrix using the auto
     * diff mechanism */
    void constraintAndJacobian(bool Constraint_only = false) final;
    /*! Auxiliary function that extract the idx-th column of the jacobian matrix from the AD
     * Constraint vector */
    template <int idx>
    inline void _updateJacobianAD();
  };
} // namespace srrg2_solver
