#include "odometry_message.h"

namespace srrg2_core {
  OdometryMessage::OdometryMessage(const std::string& topic_,
                                   const std::string& frame_id_,
                                   const int& seq_,
                                   const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(child_frame, ""),
    SETUP_PROPERTY(pose, Isometry3f::Identity()),
    SETUP_PROPERTY(linear_velocity, Vector3f(0, 0, 0)),
    SETUP_PROPERTY(angular_velocity, Vector3f(0, 0, 0)),
    SETUP_PROPERTY(pose_covariance, Matrix6f::Identity()),
    SETUP_PROPERTY(twist_covariance, Matrix6f::Identity()) {
  }

} // namespace srrg2_core
