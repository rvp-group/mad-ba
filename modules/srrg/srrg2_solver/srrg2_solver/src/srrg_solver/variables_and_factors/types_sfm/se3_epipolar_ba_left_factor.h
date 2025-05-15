#pragma once
#include "srrg_solver/variables_and_factors/types_3d/variable_se3.h"
#include "srrg_solver/solver_core/factor.h"
#include <srrg_pcl/point_types.h>
#include <srrg_data_structures/correspondence.h>
#include "srrg_solver/solver_core/factor_correspondence_driven.h"
#include "srrg_solver/solver_core/factor_correspondence_driven_dynamic.h"
#include <srrg_geometry/kahan_adder.h>
#include "se3_epipolar_ba_factor_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;


  template <typename VariableType_>
  class SE3EpipolarBALeftFactor_ : public Factor_<VariablePtrTuple_<VariableType_,
                                                                    VariableType_> >,
                                   public SE3EpipolarBAFactorBase{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = Factor_<VariablePtrTuple_<VariableType_, VariableType_> >;
    static constexpr int ErrorDim = 1;
    using InformationMatrixType = Eigen::Matrix<float, ErrorDim, ErrorDim>;
    using JacobianMatrixType = Eigen::Matrix<float, 1, 6>;
    


    // calculares the terms of H, b and the chi in the internal adders
    void compute(bool chi_only = false, bool force = false) override;

    // flushes the results of the summatins in the system matrix
    void finalizeCompute(bool chi_only) override;
    inline bool isValid() const override {return _is_valid;}
    int measurementDim() const override {return 1;}
    
  protected:
    // jacobians are opposite with left multiplication
    // we return only the Jacobian of the first variable
    inline void errorAndJacobian(float& e,
                                 JacobianMatrixType& Ji,
                                 const Vector3f& zi,
                                 const Vector3f& zj,
                                 bool error_only) const;
    
    // prepares cached strucutres for the jacobian calculations,
    // clears the adders, to be called first in the derived classes
    // returns false if nothing to do
    bool _resetCompute(bool chi_only);


    // cached crap that is updated in resetCompute
    // speeds up calculation of jacobians and the errors
    Eigen::Matrix3f _E;
    Eigen::Matrix<float, 9, 6> _Je;
    bool _is_valid=true; // toggled to false if translation between two poses is too small
    KahanAdder_<Matrix6f> _H_adder;
    KahanAdder_<Vector6f> _b_adder;
    KahanAdder_<double> _chi_adder;
    KahanAdder_<double> _chi_kernel_adder;
    
    
  };

  class SE3EpipolarBAEulerLeftFactor: public SE3EpipolarBALeftFactor_<VariableSE3EulerLeft> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void resetCompute(bool chi_only) override;
  };
    
  using SE3EpipolarBAEulerLeftFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3EpipolarBAEulerLeftFactor,Point3fVectorCloud, Point3fVectorCloud>;

  using SE3EpipolarBAEulerLeftFactorCorrespondenceDrivenDynamic =
    FactorCorrespondenceDrivenDynamic_<SE3EpipolarBAEulerLeftFactor>;

  class SE3EpipolarBAQuaternionLeftFactor: public SE3EpipolarBALeftFactor_<VariableSE3QuaternionLeft> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void resetCompute(bool chi_only) override;
  };
    
  using SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3EpipolarBAQuaternionLeftFactor,Point3fVectorCloud, Point3fVectorCloud>;

  using SE3EpipolarBAQuaternionLeftFactorCorrespondenceDrivenDynamic =
    FactorCorrespondenceDrivenDynamic_<SE3EpipolarBAQuaternionLeftFactor>;

} // namespace srrg2_solver

