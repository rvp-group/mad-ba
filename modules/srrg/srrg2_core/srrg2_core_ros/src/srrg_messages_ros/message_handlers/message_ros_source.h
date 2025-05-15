#pragma once
// srrg ros stuff
#include "srrg_converters/converter.h"
#include <ros/ros.h>

// srrg srrg2 stuff
#include "srrg_messages/instances.h"

namespace srrg2_core_ros {
  void rosInit();

  class MessageROSSource : public srrg2_core::MessageSourceBase {
  public:
    PARAM_VECTOR(srrg2_core::PropertyVector_<std::string>,
                 topics,
                 "list of the topics you want to convert",
                 0);
    PARAM(srrg2_core::PropertyBool, verbose, "verbose", false, 0);

    MessageROSSource();
    ~MessageROSSource();

    // ds interface
  public:
    MessageSourceBase* getRootSource() override;
    srrg2_core::BaseSensorMessagePtr getMessage() override;
    void open() override;
    void close();

    bool ok() {
      return ros::ok();
    }

    bool cmdOpen(std::string& response);
    bool cmdClose(std::string& response);
    // ds helpers
  protected:
    void _cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_,
                             const std::string& topic_name);
    void _imageCallback(const sensor_msgs::ImageConstPtr& msg_, const std::string& topic_name);
    void _imuCallback(const sensor_msgs::ImuConstPtr& msg_, const std::string& topic_name);
    void _laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg_,
                            const std::string& topic_name);
    void _odometryCallback(const nav_msgs::OdometryConstPtr& msg_, const std::string& topic_name);
    void _pointStampedCallback(const geometry_msgs::PointStampedConstPtr& msg_,
                               const std::string& topic_name);
    void _rangeCallback(const sensor_msgs::RangeConstPtr& msg_, const std::string& topic_name);
    void _tfMessageCallback(const tf2_msgs::TFMessageConstPtr& msg_, const std::string& topic_name);
    void _twistStampedCallback(const geometry_msgs::TwistStampedConstPtr& msg_,
                               const std::string& topic_name);
    void _pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg_,
                              const std::string& topic_name);
    void _poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg_,
                              const std::string& topic_name);
    void
    _poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_,
                                       const std::string& topic_name);
    void _pathCallback(const nav_msgs::PathConstPtr& msg_, const std::string& topic_name);
    void _gridMapCallback(const grid_map_msgs::GridMapConstPtr& msg_,
                          const std::string& topic_name);

  protected:
    // srrg the map holds the messages by timestamp
    std::multimap<double, srrg2_core::BaseSensorMessagePtr> _message_queue;
    ros::NodeHandle _node_handle;
    std::vector<ros::Subscriber> _subscribers;
    ros::master::V_TopicInfo topics_to_process;
    uint64_t _sequence_number = 0;
  };
} // namespace srrg2_core_ros
