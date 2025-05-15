#pragma once
#include "base_sensor_message.h"
#include "pose_message.h"
#include "srrg_property/property_serializable.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_property/property_eigen.h"

namespace srrg2_core {
  class PoseStampedMessage: public BaseSensorMessage {
  public:
    PoseStampedMessage(const std::string& topic_    = "",
                       const std::string& frame_id_ = "",
                       const int& seq_              = -1,
                       const double& timestamp_     = -1);

    PropertySerializable_<PoseMessage> pose;
  };

  using PoseStampedMessagePtr = std::shared_ptr<PoseStampedMessage>;
}
