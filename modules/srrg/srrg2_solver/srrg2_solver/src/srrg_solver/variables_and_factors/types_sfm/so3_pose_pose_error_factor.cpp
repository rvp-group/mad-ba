#include "so3_pose_pose_error_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  using namespace std;
  
  void SO3RelaxedPosePoseErrorFactor::errorAndJacobian(bool error_only_) {
    // ia take objects - static cast for speeeeeed
    auto * v_from = _variables.at<0>();
    auto * v_to   = _variables.at<1>();

    
    Eigen::Matrix <float, 9, 9> Jm;
    Jm.setZero();
    Jm.block<3,3>(0,0)=_measurement.transpose();
    Jm.block<3,3>(3,3)=_measurement.transpose();
    Jm.block<3,3>(6,6)=_measurement.transpose();
    const auto v_flat=VariableSO3Relaxed::flatten(v_from->estimate());
    // cerr << "Jm" << endl << Jm << endl;
    // cerr << "flat X: " << v_flat.transpose() << endl;
    const auto prediction = Jm * v_flat;
    //cerr << "pred: " << endl << prediction.transpose() << endl;
    //auto R_est=v_from->estimate()*_measurement;
    //cerr << "R_dest: " << endl << v_to->estimate() << endl;
    //cerr << "R_est: " << endl << R_est << endl;
    _e = prediction - VariableSO3Relaxed::flatten(v_to->estimate());
    //cerr << "e: " << _e.transpose() << endl;
    //exit(0);

    if (error_only_) 
      return;
    
    jacobian<0>() = Jm;
    jacobian<1>() = - Eigen::Matrix <float, 9, 9>::Identity();
      
  }


  
  SO3PosePoseQuaternionRightErrorFactorAD::ADErrorVectorType
  SO3PosePoseQuaternionRightErrorFactorAD::operator()(VariableTupleType& vars)  {
    const auto& to=vars.at<1>()->adEstimate();
    Matrix3_<DualValuef> err = _ad_inverse_measurement*vars.at<0>()->adEstimate().transpose()*to;
    return geometry3d::r2nq(err);
  }
}
