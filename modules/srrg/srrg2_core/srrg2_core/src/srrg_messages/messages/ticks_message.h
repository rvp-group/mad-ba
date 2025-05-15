#pragma once

#include <map>

#include "base_sensor_message.h"
#include "srrg_property/property_serializable.h"

namespace srrg2_core {

  struct Tick: public Serializable {
    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;

    std::string joint_name;
    int max_count;
    int count;
  };

  class TicksMessage: public BaseSensorMessage {
  public:

    TicksMessage(const std::string& topic_ = "",
                 const std::string& frame_id_ = "",
                 const int& seq_ = -1,
                 const double& timestamp_ = -1.0);

    PropertySerializableVector_<Tick> ticks;

  };

  using TickMessagePtr = std::shared_ptr<Tick>;


}  // namespace srrg2_core
