#include "message_ros_sink.h"
#include "message_ros_source.h"
#include "srrg_converters/converter.h"
#include <srrg_config/configurable_command.h>

#define CONVERT_AND_PUBLISH(SRRG_MESSAGE)                                                      \
  {                                                                                            \
    if (auto ros_msg =                                                                         \
          srrg2_core_ros::Converter::convert(std::dynamic_pointer_cast<SRRG_MESSAGE>(msg_))) { \
      pubs_it->second.publish(ros_msg);                                                        \
      return true;                                                                             \
    }                                                                                          \
  }

#define TRY_ADVERTISE_TOPIC(SRRG_CLASS_NAME, ROS_MESSAGE)                                 \
  {                                                                                       \
    const std::string& topic = param_topics.value(i);                                     \
    if (param_types.value(i) == #SRRG_CLASS_NAME) {                                       \
      std::cerr << "advertising " << #SRRG_CLASS_NAME << " topic:" << topic << std::endl; \
      _publishers.insert(std::make_pair(topic, _nh.advertise<ROS_MESSAGE>(topic, 1)));    \
      continue;                                                                           \
    }                                                                                     \
  }
using namespace srrg2_core;

namespace srrg2_core_ros {

  MessageROSSink::MessageROSSink() {
    addCommand(new srrg2_core::ConfigurableCommand_<MessageROSSink,
                                                    typeof(&MessageROSSink::cmdOpen),
                                                    std::string>(
      this, "open", "starts receiving messages", &MessageROSSink::cmdOpen));

    addCommand(new srrg2_core::ConfigurableCommand_<MessageROSSink,
                                                    typeof(&MessageROSSink::cmdClose),
                                                    std::string>(
      this, "close", "starts receiving messages", &MessageROSSink::cmdClose));

    rosInit();
  }

  bool MessageROSSink::cmdOpen(std::string& response) {
    open();
    response = "sink opened";
    return true;
  }

  bool MessageROSSink::cmdClose(std::string& response) {
    close();
    response = "sink closed";
    return true;
  }

  void MessageROSSink::open() {
    rosInit();
    this->close();
    const std::size_t size = param_types.size();

    if (size != param_topics.size()) {
      throw std::runtime_error(
        "MessageROSSink::open|ERROR, topics and types have different length");
    }

    if (size == 0) {
      throw std::runtime_error(
        "MessageROSSink::open|ERROR, no topic/msg_type specified in the configuration");
    }

    for (std::size_t i = 0; i < size; ++i) {
      // ia for each message try to check if the topic[i] and type[i] correspond.
      // ia if this is the case, create a publisher
      // ia WARNING, no namespace should be used in this macro, since it's used to create a string
      // ia from it
      TRY_ADVERTISE_TOPIC(CameraInfoMessage, sensor_msgs::CameraInfo);
      TRY_ADVERTISE_TOPIC(ImageMessage, sensor_msgs::Image);
      TRY_ADVERTISE_TOPIC(IMUMessage, sensor_msgs::Imu);
      TRY_ADVERTISE_TOPIC(LaserMessage, sensor_msgs::LaserScan);
      TRY_ADVERTISE_TOPIC(OdometryMessage, nav_msgs::Odometry);
      TRY_ADVERTISE_TOPIC(PointStampedMessage, geometry_msgs::PointStamped);
      TRY_ADVERTISE_TOPIC(RangeMessage, sensor_msgs::Range);
      TRY_ADVERTISE_TOPIC(TransformEventsMessage, tf2_msgs::TFMessage);
      TRY_ADVERTISE_TOPIC(TwistStampedMessage, geometry_msgs::TwistStamped);
      TRY_ADVERTISE_TOPIC(PointCloud2Message, sensor_msgs::PointCloud2);
      TRY_ADVERTISE_TOPIC(JointsMessage, sensor_msgs::JointState);
      TRY_ADVERTISE_TOPIC(NavsatFixMessage, sensor_msgs::NavSatFix);
      TRY_ADVERTISE_TOPIC(PoseArrayMessage, geometry_msgs::PoseArray);
      TRY_ADVERTISE_TOPIC(PathMessage, nav_msgs::Path);
      TRY_ADVERTISE_TOPIC(GridMapMessage, grid_map_msgs::GridMap);
      TRY_ADVERTISE_TOPIC(PlannerStatusMessage, srrg2_core_ros::PlannerStatusMessage);

      std::cerr << "MessageROSSink::open|failed to advertise message of type [ "
                << FG_YELLOW(param_types.value(i)) << " ]"
                << " on topic [ " << FG_YELLOW(param_topics.value(i)) << " ]\n";
      throw std::runtime_error(
        "MessageROSSink::open|ERROR, message is unregistered in the ros sink");
    }

    _is_open = true;
  }

  void MessageROSSink::close() {
    if (!_is_open) {
      return;
    }
    for (auto& p : _publishers) {
      p.second.shutdown();
    }
    _is_open = false;
  }

  bool MessageROSSink::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    if (!msg_) {
      return false;
    }
    auto pubs_it = _publishers.find(msg_->topic.value());
    if (pubs_it == _publishers.end()) {
      //#ifndef NDEBUG
      std::cerr << "MessageROSSink::putMessage|WARNING, cannot find publisher for topic [ "
                << FG_YELLOW(msg_->topic.value()) << " ]\n";
      //#endif
      return false;
    }

    CONVERT_AND_PUBLISH(CameraInfoMessage);
    CONVERT_AND_PUBLISH(ImageMessage);
    CONVERT_AND_PUBLISH(IMUMessage);
    CONVERT_AND_PUBLISH(LaserMessage);
    CONVERT_AND_PUBLISH(OdometryMessage);
    CONVERT_AND_PUBLISH(PointStampedMessage);
    CONVERT_AND_PUBLISH(RangeMessage);
    CONVERT_AND_PUBLISH(TransformEventsMessage);
    CONVERT_AND_PUBLISH(TwistStampedMessage);
    CONVERT_AND_PUBLISH(PointCloud2Message);
    CONVERT_AND_PUBLISH(JointsMessage);
    CONVERT_AND_PUBLISH(NavsatFixMessage);
    CONVERT_AND_PUBLISH(PoseArrayMessage);
    CONVERT_AND_PUBLISH(PathMessage);
    CONVERT_AND_PUBLISH(GridMapMessage);
    CONVERT_AND_PUBLISH(srrg2_core::PlannerStatusMessage);

    //#ifndef NDEBUG
    std::cerr << "MessageROSSink::putMessage|WARNING, cannot publish message [ "
              << msg_->className() << " ] on topic [ " << FG_YELLOW(msg_->topic.value()) << " ]\n";
    //#endif

    return false;
  }

  MessageROSSink::~MessageROSSink() {
    if (_is_open) {
      this->close();
    }
    if (_nh.ok()) {
      _nh.shutdown();
    }
  }

} // namespace srrg2_core_ros
