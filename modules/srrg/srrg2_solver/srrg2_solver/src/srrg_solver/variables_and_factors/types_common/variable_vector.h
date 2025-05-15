#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  /** @brief 2D Point Variable.
   */
  template <int Dim_>
  struct NestedVectorDeclarator_ {
    template <typename Scalar_>
    using VectorType = Vector_<Scalar_, Dim_>;
  };

  template <int Dim_>
  class VariableVector_
    : public Variable_<Dim_, NestedVectorDeclarator_<Dim_>::template VectorType> {
  public:
    static constexpr int Dim = Dim_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType = VariableVector_<Dim_>;
    using EstimateType     = typename NestedVectorDeclarator_<Dim_>::template VectorType<float>;
    using ADEstimateType = typename NestedVectorDeclarator_<Dim_>::template VectorType<DualValuef>;

    virtual ~VariableVector_() = default;
    virtual void setZero() override;
    virtual void applyPerturbation(const typename BaseVariableType::EstimateType& pert) override;
  };

} // namespace srrg2_solver
