#include "imu_message.h"

namespace srrg2_core {
  IMUMessage::IMUMessage(const std::string& topic_,
                         const std::string& frame_id_,
                         const int& seq_,
                         const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(orientation, Matrix3f::Identity()),
    SETUP_PROPERTY(orientation_covariance, Matrix3f::Identity()),
    SETUP_PROPERTY(angular_velocity, Vector3f(0, 0, 0)),
    SETUP_PROPERTY(angular_velocity_covariance, Matrix3f::Identity()),
    SETUP_PROPERTY(linear_acceleration, Vector3f(0, 0, 0)),
    SETUP_PROPERTY(linear_acceleration_covariance, Matrix3f::Identity()),
    SETUP_PROPERTY(rate_hz, 1000.0),
    SETUP_PROPERTY(gyroscope_noise_density, 1e-5),
    SETUP_PROPERTY(gyroscope_random_walk, 1e-5),
    SETUP_PROPERTY(accelerometer_noise_density, 1e-3),
    SETUP_PROPERTY(accelerometer_random_walk, 1e-3) {
  }

} // namespace srrg2_core
