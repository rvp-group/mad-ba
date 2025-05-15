#pragma once
#include "base_sensor_message.h"
#include "srrg_property/property_eigen.h"

namespace srrg2_core {

  class CameraInfoMessage : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraInfoMessage(const std::string& topic_    = "",
                      const std::string& frame_id_ = "",
                      const int& seq_              = -1,
                      const double& timestamp_     = -1);

    inline void setCameraMatrixFromSphericalImageParameters(const Vector4f& params_vector_) {
      Eigen::Matrix3f& cm = camera_matrix.value();
      cm << params_vector_(2), 0, params_vector_(0) * params_vector_(2), 0, params_vector_(3),
        params_vector_(1) * params_vector_(3), 0, 0, 1;
    }

    PropertyUnsignedInt rows;
    PropertyUnsignedInt cols;
    PropertyFloat depth_scale;
    PropertyString projection_model;
    PropertyString distortion_model;
    PropertyEigen_<Matrix3f> camera_matrix;
    PropertyEigen_<Eigen::VectorXd> distortion_coefficients;
  };

  using CameraInfoMessagePtr = std::shared_ptr<CameraInfoMessage>;

} // namespace srrg2_core
