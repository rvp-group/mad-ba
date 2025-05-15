
#include <srrg_geometry/geometry3d.h>
namespace srrg2_solver {
  using namespace srrg2_core;

  using Matrix15f = Eigen::Matrix<float, 15, 15>;
  class PreintegratedImuMeasurements {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PreintegratedImuMeasurements() {
    }

    inline void setBiasAccelerometer(const Vector3f& b_acc_) {
      _bias_acc = b_acc_;
    }

    inline void setBiasGyroscope(const Vector3f& b_omega_) {
      _bias_omega = b_omega_;
    }

    inline Vector3f biasAccelerometer() const {
      return _bias_acc;
    }

    inline Vector3f biasGyroscope() const {
      return _bias_omega;
    }

    inline void setNoiseGyroscope(const Vector3f& sigma_gyro) {
      _variance_noise.head<3>() = sigma_gyro.cwiseAbs2();
    }

    inline void setNoiseAccelerometer(const Vector3f& sigma_acc) {
      _variance_noise.tail<3>() = sigma_acc.cwiseAbs2();
    }

    inline void setNoiseBiasAccelerometer(const Vector3f& sigma_bias_acc) {
      _variance_noise_bias.tail<3>() = sigma_bias_acc.cwiseAbs2();
    }

    inline void setNoiseBiasGyroscope(const Vector3f& sigma_bias_gyro) {
      _variance_noise_bias.head<3>() = sigma_bias_gyro.cwiseAbs2();
    }

    // ldg accessor for factor vomito diarroico

    inline Matrix3f delta_Rij() const {
      return _delta_Rij;
    }

    inline Vector3f delta_pij() const {
      return _delta_pij;
    }

    inline Vector3f delta_vij() const {
      return _delta_vij;
    }

    inline float delta_t() const {
      return _dt;
    }

    inline Matrix3f dRij_dbg() const {
      return _dRij_dbg;
    }

    inline Matrix3f dpij_dba() const {
      return _dpij_dba;
    }

    inline Matrix3f dpij_dbg() const {
      return _dpij_dbg;
    }

    inline Matrix3f dvij_dba() const {
      return _dvij_dba;
    }

    inline Matrix3f dvij_dbg() const {
      return _dvij_dbg;
    }

    inline Vector6f sigmaNoise() const {
      return _variance_noise.cwiseSqrt();
    }

    inline Matrix15f covarianceMatrix() const {
      return _covariance;
    }

    inline Matrix15f informationMatrix() const {
      return _covariance.inverse();
    }

    void reset();
    void integrate(const Vector3f& acc_, const Vector3f& omega_, const float& dt_);
    PreintegratedImuMeasurements& operator=(const PreintegratedImuMeasurements& other);

  protected:
    // update jacobians wrt bias and covariance propagation for preintegrated measurements
    void _update(const Vector3f& acc_,
                 const Vector3f& delta_theta_,
                 const Matrix3f& delta_R_,
                 const float& dt_);

    Vector3f _bias_acc    = Vector3f::Zero(); // bias acceleration
    Vector3f _bias_omega  = Vector3f::Zero(); // bias angular vel
    Vector3f _noise_acc   = Vector3f::Zero();
    Vector3f _noise_omega = Vector3f::Zero();

    Matrix3f _delta_Rij = Matrix3f::Identity();
    Vector3f _delta_pij = Vector3f::Zero();
    Vector3f _delta_vij = Vector3f::Zero();
    float _dt           = 0.f;

    // derivatives wrt to biases for biases update
    Matrix3f _dRij_dbg       = Matrix3f::Zero(); // like derivative of Rij wrt to bias gyro
    Matrix3f _dpij_dba       = Matrix3f::Zero();
    Matrix3f _dpij_dbg       = Matrix3f::Zero();
    Matrix3f _dvij_dba       = Matrix3f::Zero();
    Matrix3f _dvij_dbg       = Matrix3f::Zero();
    Matrix15f _covariance = Matrix15f::Zero();
    Vector6f _variance_noise = Vector6f::Zero(); // respectively noise for accelerometer and gyro
    // respectively noise for accelerometer and gyro of biases
    Vector6f _variance_noise_bias = Vector6f::Zero();
  };

} // namespace srrg2_solver
