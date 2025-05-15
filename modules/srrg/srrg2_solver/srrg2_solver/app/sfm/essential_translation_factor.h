#pragma once
#include "srrg_solver/variables_and_factors/types_3d/variable_point3.h"
#include "srrg_solver/solver_core/error_factor.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  
  class EssentialTranslationFactor : public ErrorFactor_<3, VariablePoint3, VariablePoint3 >{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ErrorFactor_<3, VariablePoint3, VariablePoint3 >;
    static constexpr int ErrorDim = BaseType::ErrorDim;
    using JacobianMatrixType = Eigen::Matrix<float, 3, 3>;
    Eigen::Matrix3f Ri, Rj;
    Eigen::Vector3f tij_essential;
    void resetCompute(bool chi_only) override;

  protected:
    // jacobians are opposite with left multiplication
    // we return only the Jacobian of the first variable
    void errorAndJacobian(bool error_only) override;

    // cached crap that is updated in resetCompute
    // speeds up calculation of jacobians and the errors
    Eigen::Matrix3f _E;
    Eigen::Matrix<float, 9, 3> _Je;
  };

}
