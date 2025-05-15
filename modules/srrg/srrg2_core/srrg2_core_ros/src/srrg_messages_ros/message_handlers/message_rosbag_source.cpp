#include "message_rosbag_source.h"
#include "message_ros_source.h"
#include "srrg_config/configurable.h"
#include "srrg_converters/converter.h"
#include "srrg_messages/message_handlers/message_file_source.h"
#include "srrg_messages/messages/base_sensor_message.h"
#include "srrg_property/property_vector.h"

#define CONVERT(ROS_MESSAGE)                                                    \
  {                                                                             \
    if (ROS_MESSAGE##ConstPtr ros_msg = _view_it->instantiate<ROS_MESSAGE>()) { \
      srrg_message = srrg2_core_ros::Converter::convert(ros_msg);               \
    }                                                                           \
  }

namespace srrg2_core_ros {
  using namespace srrg2_core;
  using namespace srrg2_core_ros;

  MessageROSBagSource::MessageROSBagSource() {
    rosInit();
  }

  void MessageROSBagSource::open() {
    _view.reset(nullptr);
    std::cerr << "opening bag... ";
    _bag.open(param_filename.value());
    std::cerr << "done" << std::endl;
    if (param_topics.size()) {
      _view.reset(new rosbag::View(_bag, rosbag::TopicQuery(param_topics.value())));
    } else {
      _view.reset(new rosbag::View(_bag));
    }
    if (_view) {
      _view_it = _view->begin();
    }
  }

  void MessageROSBagSource::open(const std::string& bag_filename_) {
    MessageFileSourceBase::open(bag_filename_);
  }

  void MessageROSBagSource::reset() {
    if (_view) {
      _view_it = _view->begin();
    }
  }

  void MessageROSBagSource::close() {
    MessageFileSourceBase::close();
    _bag.close();
    _view.reset();
  }

  BaseSensorMessagePtr MessageROSBagSource::getMessage() {
    if (this->_file_changed_flag) {
      open(param_filename.value());
    }
    if (!_view) {
      return nullptr;
    }
    while (_view_it != _view->end() && _running) {
      const std::string& topic_name = _view_it->getTopic();
      const std::string& data_type  = _view_it->getDataType();
      // ds generic SRRG message
      PropertyContainerSerializablePtr srrg_message = nullptr;

      // ds parse depending on topic name TODO enable automatic conversion for registered topics in
      // Converter?

      // srrg this way you don't have to care about the topic
      //      the topic you want to subscribe are chosen in the config file
      //      and the message contains the topic field so you can discriminate in case of different
      //      topics with same datatype
      CONVERT(sensor_msgs::CameraInfo);
      CONVERT(sensor_msgs::Image);
      CONVERT(sensor_msgs::Imu);
      CONVERT(sensor_msgs::Range);
      CONVERT(sensor_msgs::LaserScan);
      CONVERT(tf2_msgs::TFMessage);
      CONVERT(nav_msgs::Odometry);
      CONVERT(geometry_msgs::PointStamped);
      CONVERT(geometry_msgs::TwistStamped);
      CONVERT(sensor_msgs::PointCloud2);
      CONVERT(sensor_msgs::JointState);
      CONVERT(sensor_msgs::NavSatFix);

      ++_view_it;
      if (!srrg_message) {
        if (param_verbose.value()) {
          std::cerr << "WARNING: ignoring non-configured conversion for data type [" << data_type
                    << "]"
                    << " over topic " << topic_name << std::endl;
        }
        continue;
      }
      // ds convert input and attempt downcast
      if (BaseSensorMessagePtr srrg_base_message =
            std::dynamic_pointer_cast<BaseSensorMessage>(srrg_message)) {
        srrg_base_message->topic.setValue(topic_name);
        return srrg_base_message;
      }
    }
    return nullptr;
  }

} // namespace srrg2_core_ros
