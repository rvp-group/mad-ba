#include "preintegrated_imu_measurements.h"
namespace srrg2_solver {
  using namespace srrg2_core;

  void PreintegratedImuMeasurements::integrate(const Vector3f& acc_,
                                               const Vector3f& omega_,
                                               const float& dt_) {
    if (dt_ <= 0) {
      throw std::runtime_error("PreintegratedImuMeasurements::integrate: dt <=0");
    }

    // correct bias in the sensor frame
    const Vector3f acc_correct   = acc_ - _bias_acc;
    const Vector3f omega_correct = omega_ - _bias_omega;
    // TODO compensate for sensor-body displacement if needed ?????
    // TODO use extrinsic offset sensor-body eventually
    // calculate rotation increment between time slots
    const Vector3f delta_theta = omega_correct * dt_;
    const Matrix3f delta_R     = geometry3d::expMapSO3(delta_theta);
    _update(acc_correct, delta_theta, delta_R, dt_);
    const float dt2 = dt_ * dt_;
    // update preintegrated measurement
    const Vector3f delta_p = _delta_vij * dt_ + 0.5 * _delta_Rij * acc_correct * dt2;
    _delta_pij += delta_p;
    const Vector3f delta_v = _delta_Rij * acc_correct * dt_;
    _delta_vij += delta_v;
    _delta_Rij *= delta_R;
    fixRotation(_delta_Rij);
    _dt += dt_;
  }

  void PreintegratedImuMeasurements::_update(const Vector3f& acc_,
                                             const Vector3f& delta_theta_,
                                             const Matrix3f& delta_R_,
                                             const float& dt_) {
    // const Vector3f acceleration_in_prev_frame = _delta_Rij * acc_;
    // const Matrix3f skew_acceleration          = geometry3d::skew(acceleration_in_prev_frame);
    const Matrix3f skew_acceleration          = _delta_Rij * geometry3d::skew(acc_);
    const Matrix3f J_acc_wrt_bg               = skew_acceleration * _dRij_dbg;
    const Matrix3f dR_increment               = geometry3d::jacobianExpMapSO3(delta_theta_);
    // delta translation measurement wrt bias accelaration and gyro
    // _delta_Rij is the previous relative rotation
    float half_dt2 = 0.5 * dt_ * dt_;
    _dpij_dba += _dvij_dba * dt_ - half_dt2 * _delta_Rij;
    _dpij_dbg += _dvij_dbg * dt_ + half_dt2 * J_acc_wrt_bg;
    // delta velocity measurement wrt bias acc
    // _delta_Rij is the previous relative rotation
    _dvij_dba -= _delta_Rij * dt_;
    _dvij_dbg += J_acc_wrt_bg * dt_;
    // delta rotation measurement wrt bias acc
    _dRij_dbg = delta_R_.transpose() * _dRij_dbg - dR_increment * dt_;

    // covariance propagation
    Matrix15f A;
    A.setIdentity();
    A.block<3, 3>(0, 0) = delta_R_.transpose();
    A.block<3, 3>(3, 0) = -skew_acceleration * dt_;
    A.block<3, 3>(6, 0) = -skew_acceleration * half_dt2;
    A.block<3, 3>(6, 3) = Matrix3f::Identity() * dt_;

    Eigen::Matrix<float, 15, 6> B;
    B.setZero();
    B.block<3, 3>(0, 0) = dR_increment * dt_;
    B.block<3, 3>(3, 3) = _delta_Rij * dt_;
    B.block<3, 3>(6, 3) = _delta_Rij * half_dt2;

    Matrix3f J_position_wrt_ba    = -B.block<3, 3>(6, 3);
    Matrix3f J_velocity_wrt_ba    = -B.block<3, 3>(3, 3);
    Matrix3f J_orientation_wrt_bg = -B.block<3, 3>(0, 0);
    A.block<3, 3>(0, 9)           = J_orientation_wrt_bg;
    A.block<3, 3>(3, 12)          = J_velocity_wrt_ba;
    A.block<3, 3>(6, 12)          = J_position_wrt_ba;

    const Matrix6f covariance_measurements = _variance_noise.asDiagonal();
    const Matrix6f covariance_biases       = _variance_noise_bias.asDiagonal();
    Matrix15f sigma_u = (1 / dt_) * B * covariance_measurements * B.transpose();
    sigma_u.block<6, 6>(9, 9) = dt_ * covariance_biases;

    _covariance = A * _covariance * A.transpose();
    _covariance.noalias() += sigma_u;
  }

  PreintegratedImuMeasurements& PreintegratedImuMeasurements::
  operator=(const PreintegratedImuMeasurements& other) {
    this->_bias_acc    = other.biasAccelerometer();
    this->_bias_omega  = other.biasGyroscope();
    this->_delta_Rij   = other.delta_Rij();
    this->_delta_vij   = other.delta_vij();
    this->_delta_pij   = other.delta_pij();
    this->_dRij_dbg    = other.dRij_dbg(); // like derivative of Rij wrt to bias gyro
    this->_dpij_dba    = other.dpij_dba();
    this->_dpij_dbg    = other.dpij_dbg();
    this->_dvij_dba    = other.dvij_dba();
    this->_dvij_dbg    = other.dvij_dbg();
    this->_dt          = other.delta_t();
    this->_covariance  = other.covarianceMatrix();
    this->_variance_noise = other.sigmaNoise().cwiseAbs2();
    return *this;
  }

  void PreintegratedImuMeasurements::reset() {
    _delta_Rij.setIdentity();
    _delta_pij.setZero();
    _delta_vij.setZero();
    _dt = 0.f;
    // clear jacobians
    _dRij_dbg.setZero();
    _dpij_dba.setZero();
    _dpij_dbg.setZero();
    _dvij_dba.setZero();
    _dvij_dbg.setZero();
    _covariance.setZero();
  }

} // namespace srrg2_solver
