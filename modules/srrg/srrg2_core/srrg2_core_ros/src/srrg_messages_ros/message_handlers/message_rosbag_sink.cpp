#include "message_rosbag_sink.h"
#include <srrg_converters/converter.h>
#include "message_ros_sink.h"

#define CONVERT(SRRG_MESSAGE)                                                               \
  {                                                                                         \
    if (auto ros_msg = srrg2_core_ros::Converter::convert(std::dynamic_pointer_cast<SRRG_MESSAGE>(msg_))) { \
      _bag.write(msg_->topic.value(), ros::Time(msg_->timestamp.value()), ros_msg); \
      return true;                                                                          \
    }                                                                                       \
  }

namespace srrg2_core_ros {

  MessageROSBagSink::MessageROSBagSink() {
  }

  void MessageROSBagSink::open(const std::string& filename_) {
    MessageFileSinkBase::open(filename_);
  }

  void MessageROSBagSink::open() {
    this->close();
    _bag.open(param_filename.value(), rosbag::bagmode::Write);
    if (param_verbose.value()) {
      std::cerr << "MessageROSBagSink::open| bag [" << param_filename.value() << "] open" << std::endl;
    }
    _is_open = true;
    _file_changed_flag=false;
  }

  void MessageROSBagSink::close() {
    if (!_is_open) {
      return;
    }
    _bag.close();
    if (param_verbose.value()) {
      std::cerr << "MessageROSBagSink::open| bag closed" << std::endl;
    }
    _is_open = false;
  }

  bool MessageROSBagSink::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    if (_file_changed_flag)
      open();
    
    if (!msg_) {
      return false;
    }

    CONVERT(srrg2_core::CameraInfoMessage);
    CONVERT(srrg2_core::ImageMessage);
    CONVERT(srrg2_core::IMUMessage);
    CONVERT(srrg2_core::LaserMessage);
    CONVERT(srrg2_core::RangeMessage);
    CONVERT(srrg2_core::TransformEventsMessage);
    CONVERT(srrg2_core::OdometryMessage);
    CONVERT(srrg2_core::PointStampedMessage);
    CONVERT(srrg2_core::TwistStampedMessage);
    CONVERT(srrg2_core::PointCloud2Message);

    return false;

  }

}
