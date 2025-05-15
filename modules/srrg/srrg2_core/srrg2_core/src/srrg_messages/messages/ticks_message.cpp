#include "ticks_message.h"

namespace srrg2_core {
  void Tick::serialize(ObjectData& odata, IdContext& context) {
    odata.setString("joint_name", joint_name);
    odata.setInt("max_count", max_count);
    odata.setInt("count", count);
  }

  void Tick::deserialize(ObjectData& odata, IdContext& context) {
    joint_name = odata.getString("joint_name");
    max_count = odata.getInt("max_count");
    count = odata.getInt("count");
  }

  TicksMessage::TicksMessage(const std::string& topic_,
                             const std::string& frame_id_,
                             const int& seq_,
                             const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY_NV(ticks) {
  }

}  // namespace srrg2_core
