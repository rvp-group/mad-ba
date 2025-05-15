#pragma once
// srrg ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
// srrg SRRG includes
#include "srrg_messages/instances.h"
#include "srrg_messages/message_handlers/message_file_source_base.h"
#include "srrg_system_utils/system_utils.h"

namespace srrg2_core_ros {

  class MessageROSBagSource : public srrg2_core::MessageFileSourceBase {
  public:
    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>,
                 topics,
                 "list of the topics you want to convert",
                 0);
    PARAM(srrg2_core::PropertyBool, verbose, "verbose", false, 0);

  public:
    MessageROSBagSource();
    virtual ~MessageROSBagSource() {
      this->close();
    }

    void open(const std::string& bag_filename_) override;
    void open() override;
    void close() override;
    void reset() override;
    srrg2_core::BaseSensorMessagePtr getMessage() override;

  protected:
    rosbag::Bag _bag;
    std::unique_ptr<rosbag::View> _view = nullptr;
    rosbag::View::iterator _view_it;
  };

  using MessageROSBagSourcePtr = std::shared_ptr<MessageROSBagSource>;
} // namespace srrg2_core_ros
