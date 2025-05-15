#pragma once
#include "base_sensor_message.h"
#include "srrg_property/property_eigen.h"

namespace srrg2_core {

  class IMUMessage : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUMessage(const std::string& topic_    = "",
               const std::string& frame_id_ = "",
               const int& seq_              = -1,
               const double& timestamp_     = -1);
    PropertyEigen_<Matrix3f> orientation;
    PropertyEigen_<Matrix3f> orientation_covariance;
    PropertyEigen_<Vector3f> angular_velocity;
    PropertyEigen_<Matrix3f> angular_velocity_covariance;
    PropertyEigen_<Vector3f> linear_acceleration;
    PropertyEigen_<Matrix3f> linear_acceleration_covariance;
    PropertyFloat rate_hz;
    PropertyFloat gyroscope_noise_density;     // [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
    PropertyFloat gyroscope_random_walk;       // [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
    PropertyFloat accelerometer_noise_density; // [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
    PropertyFloat accelerometer_random_walk;   // [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
  };

  using IMUMessagePtr = std::shared_ptr<IMUMessage>;

} // namespace srrg2_core
