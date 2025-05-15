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
  class SE3EpipolarBARightFactor_ : public Factor_<VariablePtrTuple_<VariableType_,
                                                                     VariableType_> >,
                                    public SE3EpipolarBAFactorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    // prepares cached strucutres for the jacobian calculations,
    // clears the adders
    // call it before the true resetCompute in the subsequent classes
    bool _resetCompute(bool chi_only);

    // jacobians are opposite with right multiplication
    // we return only the Jacobian of the first variable
    inline void errorAndJacobian(float& e,
                                 JacobianMatrixType& Ji,
                                 JacobianMatrixType& Jj,
                                 const Vector3f& zi,
                                 const Vector3f& zj,
                                 bool error_only) const;

    // cached crap that is updated in resetCompute
    // speeds up calculation of jacobians and the errors
    Eigen::Matrix3f _E;
    Eigen::Matrix<float, 9, 6> _Jei;
    Eigen::Matrix<float, 9, 6> _Jej;
    
    bool _is_valid=true; // toggled to false if translation between two poses is too small
    KahanAdder_<Matrix6f> _Hii_adder, _Hij_adder, _Hjj_adder;
    KahanAdder_<Vector6f> _bi_adder, _bj_adder;
    KahanAdder_<double> _chi_adder;
    KahanAdder_<double> _chi_kernel_adder;
  };

  class SE3EpipolarBAEulerRightFactor: public SE3EpipolarBARightFactor_<VariableSE3EulerRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    void resetCompute(bool chi_only) override;
  };

  class SE3EpipolarBAQuaternionRightFactor: public SE3EpipolarBARightFactor_<VariableSE3QuaternionRight> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    void resetCompute(bool chi_only) override;
  };

  using SE3EpipolarBAEulerRightFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3EpipolarBAEulerRightFactor,Point3fVectorCloud, Point3fVectorCloud>;

  using SE3EpipolarBAEulerRightFactorCorrespondenceDrivenDynamic =
    FactorCorrespondenceDrivenDynamic_<SE3EpipolarBAEulerRightFactor>;

  using SE3EpipolarBAQuaternionRightFactorCorrespondenceDriven =
    FactorCorrespondenceDriven_<SE3EpipolarBAQuaternionRightFactor,Point3fVectorCloud, Point3fVectorCloud>;

  using SE3EpipolarBAQuaternionRightFactorCorrespondenceDrivenDynamic =
    FactorCorrespondenceDrivenDynamic_<SE3EpipolarBAQuaternionRightFactor>;

} // namespace srrg2_solver

