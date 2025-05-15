#include "pose_with_covariance_stamped_message.h"
namespace srrg2_core {
  PoseWithCovarianceStampedMessage::PoseWithCovarianceStampedMessage(const std::string& topic_,
                                                       const std::string& frame_id_,
                                                       const int& seq_,
                                                       const double& timestamp_):
    PoseStampedMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(covariance, Matrix6f::Identity()){}

  PoseWithCovarianceStampedMessage::~PoseWithCovarianceStampedMessage(){}
  
}
