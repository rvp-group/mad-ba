#include "message_ros_source.h"
#include <srrg_config/configurable_command.h>
#include <srrg_system_utils/system_utils.h>
#include <sys/types.h>
#include <unistd.h>

#define ADD_SUBSCRIBER(ROS_NAMESPACE, ROS_MESSAGE, CALLBACK, QUEUE_SIZE)                         \
  {                                                                                              \
    if (ttp.datatype == std::string(#ROS_NAMESPACE) + "/" + std::string(#ROS_MESSAGE)) {         \
      ss << "subscribing to topic " << ttp.name << " publishing " << ttp.datatype << " messages" \
         << std::endl;                                                                           \
      _subscribers.push_back(_node_handle.subscribe<ROS_NAMESPACE::ROS_MESSAGE>(                 \
        ttp.name, QUEUE_SIZE, boost::bind(&MessageROSSource::CALLBACK, this, _1, ttp.name)));    \
      if (param_verbose.value()) {                                                               \
        std::cerr << ss.str() << std::endl;                                                      \
        ss.clear();                                                                              \
      }                                                                                          \
      continue;                                                                                  \
    }                                                                                            \
  }

namespace srrg2_core_ros {

  static bool ros_initialized = false;
  void rosInit() {
    if (!srrg2_core::srrg_argc) {
      throw std::runtime_error("rosInit| missing call to srrgInit. Aborting.");
    }
    if (!ros_initialized) {
      ros::init(srrg2_core::srrg_argc,
                srrg2_core::srrg_argv,
                srrg2_core::srrg_opts.c_str(),
                ros::InitOption::NoSigintHandler | ros::InitOption::AnonymousName);
    }
    ros_initialized = true;
  }

  MessageROSSource::MessageROSSource() {
    addCommand(new srrg2_core::ConfigurableCommand_<MessageROSSource,
                                                    typeof(&MessageROSSource::cmdOpen),
                                                    std::string>(
      this, "open", "starts receiving messages", &MessageROSSource::cmdOpen));

    addCommand(new srrg2_core::ConfigurableCommand_<MessageROSSource,
                                                    typeof(&MessageROSSource::cmdClose),
                                                    std::string>(
      this, "close", "starts receiving messages", &MessageROSSource::cmdClose));
    rosInit();
    _message_queue.clear();
    _subscribers.clear();
    topics_to_process.clear();
  }

  bool MessageROSSource::cmdOpen(std::string& response) {
    open();
    response = "source opened";
    return true;
  }

  bool MessageROSSource::cmdClose(std::string& response) {
    close();
    response = "source closed";
    return true;
  }

  void MessageROSSource::open() {
    rosInit();
    ros::master::V_TopicInfo all_topics;
    std::cerr << "waiting for new topics";
    ros::Rate r(10);
    int count = 0;
    while (all_topics.size() < 3 && count < 100) {
      ros::master::getTopics(all_topics);
      r.sleep();
      if (!(count % 10)) {
        std::cerr << ".";
      }
      count++;
    }
    if (count >= 100) {
      std::cerr << " no interesting topics" << std::endl;
      _is_open = false;
      return;
    }
    r.sleep();
    ros::master::getTopics(all_topics);

    if (param_verbose.value()) {
      std::cerr << "MessageROSSource::open|number of available ROS topics: " << all_topics.size()
                << std::endl;
    }

    if (param_topics.value().size() == 0) {
      topics_to_process = all_topics;
    } else {
      for (const std::string& topic_name : param_topics.value()) {
        auto it = std::find_if(
          all_topics.begin(), all_topics.end(), [topic_name](const auto& other_topic_) {
            return other_topic_.name == topic_name;
          });
        if (it != all_topics.end()) {
          topics_to_process.push_back(*it);
        }
      }
    }

    std::stringstream ss;
    for (const ros::master::TopicInfo& ttp : topics_to_process) {
      ss << "MessageROSSource::open|";
      ADD_SUBSCRIBER(sensor_msgs, CameraInfo, _cameraInfoCallback, 1);
      ADD_SUBSCRIBER(sensor_msgs, Image, _imageCallback, 1);
      ADD_SUBSCRIBER(sensor_msgs, Imu, _imuCallback, 1);
      ADD_SUBSCRIBER(nav_msgs, Odometry, _odometryCallback, 1);
      ADD_SUBSCRIBER(geometry_msgs, PointStamped, _pointStampedCallback, 1);
      ADD_SUBSCRIBER(sensor_msgs, LaserScan, _laserScanCallback, 1);
      ADD_SUBSCRIBER(sensor_msgs, Range, _rangeCallback, 1);
      ADD_SUBSCRIBER(tf2_msgs, TFMessage, _tfMessageCallback, 20);
      ADD_SUBSCRIBER(geometry_msgs, TwistStamped, _twistStampedCallback, 1);
      ADD_SUBSCRIBER(sensor_msgs, PointCloud2, _pointCloud2Callback, 1);
      ADD_SUBSCRIBER(
        geometry_msgs, PoseWithCovarianceStamped, _poseWithCovarianceStampedCallback, 100);
      ADD_SUBSCRIBER(geometry_msgs, PoseStamped, _poseStampedCallback, 100);
      ADD_SUBSCRIBER(grid_map_msgs, GridMap, _gridMapCallback, 1);
      ADD_SUBSCRIBER(nav_msgs, Path, _pathCallback, 1);
      ss << "unable to convert " << ttp.datatype << " messages (no topic registered)" << std::endl;
      std::cerr << ss.str() << std::endl;
    }
    if (param_verbose.value()) {
      std::cerr << "subscribed to " << _subscribers.size() << " topics" << std::endl;
    }

    _is_open = true;
  }

  void MessageROSSource::close() {
    if (!_is_open) {
      return;
    }
    for (ros::Subscriber& s : _subscribers) {
      s.shutdown();
    }
    _message_queue.clear();
    _subscribers.clear();
    topics_to_process.clear();
    _is_open = false;
  }

  srrg2_core::BaseSensorMessagePtr MessageROSSource::getMessage() {
    while (_message_queue.empty() && _is_open) {
      ros::spinOnce();
      usleep(10000);
    }
    if (!_is_open)
      return nullptr;

    auto it                                   = _message_queue.begin();
    srrg2_core::BaseSensorMessagePtr returned = it->second;
    _message_queue.erase(it);
    if (returned)
      return returned;
    return nullptr;
  }

  void MessageROSSource::_cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_,
                                             const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_imageCallback(const sensor_msgs::ImageConstPtr& msg_,
                                        const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_imuCallback(const sensor_msgs::ImuConstPtr& msg_,
                                      const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_odometryCallback(const nav_msgs::OdometryConstPtr& msg_,
                                           const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_pointStampedCallback(const geometry_msgs::PointStampedConstPtr& msg_,
                                               const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg_,
                                            const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_rangeCallback(const sensor_msgs::RangeConstPtr& msg_,
                                        const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_tfMessageCallback(const tf2_msgs::TFMessageConstPtr& msg_,
                                            const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg_->transforms[0].header.stamp.toSec(), msg));
  }

  void MessageROSSource::_twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg_,
                                               const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg_,
                                              const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_poseWithCovarianceStampedCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_,
    const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg_,
                                              const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_gridMapCallback(const grid_map_msgs::GridMapConstPtr& msg_,
                                          const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  void MessageROSSource::_pathCallback(const nav_msgs::PathConstPtr& msg_,
                                       const std::string& topic_name) {
    srrg2_core::BaseSensorMessagePtr msg = Converter::convert(msg_);
    if (!msg) {
      return;
    }
    msg->topic.setValue(topic_name);
    _message_queue.insert(std::make_pair(msg->timestamp.value(), msg));
  }

  srrg2_core::MessageSourceBase* MessageROSSource::getRootSource() {
    return this;
  }

  MessageROSSource::~MessageROSSource() {
    if (_is_open) {
      this->close();
    }
    if (_node_handle.ok()) {
      _node_handle.shutdown();
    }
  }

} // namespace srrg2_core_ros
