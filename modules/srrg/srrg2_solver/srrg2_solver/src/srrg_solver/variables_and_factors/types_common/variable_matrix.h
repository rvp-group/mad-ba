#pragma once
#include "srrg_solver/solver_core/variable.h"
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  /** @brief generic matrix variable. Pert vector applies to all elements
   */
  template <int Rows_, int Cols_>
  struct NestedMatrixDeclarator_ {
    template <typename Scalar_>
    using MatrixType = Eigen::Matrix<Scalar_, Rows_, Cols_>;
  };
  
  template <int Rows_, int Cols_>
  class VariableMatrix_ : public Variable_<Rows_*Cols_,
                                           NestedMatrixDeclarator_<Rows_,Cols_>:: template MatrixType > {
  public:
    static constexpr int Rows=Rows_;
    static constexpr int Cols=Cols_;
    static constexpr int Dim=Rows*Cols;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseVariableType = VariableMatrix_<Rows_,Cols_>;
    using PerturbationVectorType = typename Variable_<Rows_*Cols_,
                                             NestedMatrixDeclarator_<Rows_,Cols_>
                                             :: template MatrixType > :: PerturbationVectorType; 
    using EstimateType = typename NestedMatrixDeclarator_<Rows_,Cols_>::template MatrixType<float>;
    using ADEstimateType = typename NestedMatrixDeclarator_<Rows_,Cols_>::template MatrixType<DualValuef>;
    
      
    virtual ~VariableMatrix_() = default;
    virtual void setZero() override;
    virtual void applyPerturbation(const typename BaseVariableType::PerturbationVectorType& pert) override;
  };

} // namespace srrg2_solver
