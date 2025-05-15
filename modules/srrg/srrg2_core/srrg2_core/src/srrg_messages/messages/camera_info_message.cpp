#include "camera_info_message.h"

namespace srrg2_core {
  CameraInfoMessage::CameraInfoMessage(const std::string& topic_,
                                       const std::string& frame_id_,
                                       const int& seq_,
                                       const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(rows, 0),
    SETUP_PROPERTY(cols, 0),
    SETUP_PROPERTY(depth_scale, 1e-3),
    SETUP_PROPERTY(projection_model, "pinhole"),
    SETUP_PROPERTY(distortion_model, ""),
    SETUP_PROPERTY(camera_matrix, Matrix3f::Identity()),
    SETUP_PROPERTY(distortion_coefficients, Eigen::VectorXd()) {
  }

} // namespace srrg2_core
