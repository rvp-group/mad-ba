#pragma once
#include <rosbag/bag.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/message_handlers/message_file_sink_base.h>

namespace srrg2_core_ros {

  struct MessageROSBagSink: public srrg2_core::MessageFileSinkBase {

    MessageROSBagSink();

    void open() override;
    void open(const std::string& filename_) override;
    void close() override;
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;
  protected:
    rosbag::Bag _bag;
  };

}
