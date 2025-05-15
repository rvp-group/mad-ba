#pragma once
#include "base_sensor_message.h"
#include "pose_stamped_message.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_property/property_serializable.h"

namespace srrg2_core {
  class PathMessage: public BaseSensorMessage {
  public:
    PathMessage(const std::string& topic_    = "",
                    const std::string& frame_id_ = "",
                    const int& seq_              = -1,
                    const double& timestamp_     = -1);
    
    PropertySerializableVector_<PoseStampedMessage> poses;
    virtual ~PathMessage();
  };

  using PathMessagePtr = std::shared_ptr<PathMessage>;
}
