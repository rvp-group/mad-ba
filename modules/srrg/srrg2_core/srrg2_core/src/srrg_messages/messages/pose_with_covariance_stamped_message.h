#pragma once
#include "pose_stamped_message.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {
  class PoseWithCovarianceStampedMessage: public PoseStampedMessage {
  public:
    PoseWithCovarianceStampedMessage(const std::string& topic_    = "",
                                     const std::string& frame_id_ = "",
                                     const int& seq_              = -1,
                                     const double& timestamp_     = -1);

    PropertyEigen_<Matrix6f>   covariance;
    virtual ~PoseWithCovarianceStampedMessage();
  };

  using PoseWithCovarianceStampedMessagePtr = std::shared_ptr<PoseWithCovarianceStampedMessage>;
}
