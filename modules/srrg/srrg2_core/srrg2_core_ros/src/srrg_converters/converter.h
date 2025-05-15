#pragma once

// ds ROS messages
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <grid_map_msgs/GridMap.h>

#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <srrg2_core_ros/PlannerStatusMessage.h>
#include <tf2_msgs/TFMessage.h>

// ds SRRG messages/types
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/cmd_vel_message.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/navsat_fix_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/planner_status_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/messages/point_stamped_message.h>
#include <srrg_messages/messages/pose_array_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_messages/messages/pose_with_covariance_stamped_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/twist_stamped_message.h>
#include <srrg_property/property_container.h>

// ds TODO purge
#include "translator_utils.h"

namespace srrg2_core_ros {

  class Converter {
  public:
    // ds construction and destruction prohibited
    Converter()  = delete;
    ~Converter() = delete;

    // ds ROS to SRRG
  public:
    // ds specialized conversion interface - going down for higher specialization
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::CameraInfoConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::ImageConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::ImuConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::LaserScanConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(nav_msgs::OdometryConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr
    convert(geometry_msgs::PointStampedConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::RangeConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(tf2_msgs::TFMessageConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr
    convert(geometry_msgs::TwistStampedConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(geometry_msgs::TwistConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::PointCloud2ConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::JointStateConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(sensor_msgs::NavSatFixConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(geometry_msgs::PoseArrayConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(geometry_msgs::PoseStampedConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr
    convert(geometry_msgs::PoseWithCovarianceStampedConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(grid_map_msgs::GridMapConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr convert(nav_msgs::PathConstPtr message_in_);
    static srrg2_core::BaseSensorMessagePtr
    convert(srrg2_core_ros::PlannerStatusMessageConstPtr message_in_);
    // ds SRRG to ROS
  public:
    // srrg specialized conversion interface for the other way around ;)
    static sensor_msgs::CameraInfoPtr convert(const srrg2_core::CameraInfoMessagePtr message_in_);
    static sensor_msgs::ImagePtr convert(const srrg2_core::ImageMessagePtr message_in_);
    static sensor_msgs::ImuPtr convert(const srrg2_core::IMUMessagePtr message_in_);
    static sensor_msgs::LaserScanPtr convert(const srrg2_core::LaserMessagePtr message_in_);
    static nav_msgs::OdometryPtr convert(const srrg2_core::OdometryMessagePtr message_in_);
    static geometry_msgs::PointStampedPtr
    convert(const srrg2_core::PointStampedMessagePtr message_in_);
    static sensor_msgs::RangePtr convert(const srrg2_core::RangeMessagePtr message_in_);
    static geometry_msgs::TransformStamped convert(const srrg2_core::TransformEvent& message_in_);
    static tf2_msgs::TFMessagePtr convert(const srrg2_core::TransformEventsMessagePtr message_in_);
    static geometry_msgs::TwistStampedPtr
    convert(const srrg2_core::TwistStampedMessagePtr message_in_);
    static sensor_msgs::PointCloud2Ptr convert(const srrg2_core::PointCloud2MessagePtr message_in_);
    static sensor_msgs::JointStatePtr convert(const srrg2_core::JointsMessagePtr message_in_);
    static sensor_msgs::NavSatFixPtr convert(const srrg2_core::NavsatFixMessagePtr message_in_);
    static geometry_msgs::PoseArrayPtr convert(const srrg2_core::PoseArrayMessagePtr message_in_);
    static geometry_msgs::PoseStampedPtr
    convert(const srrg2_core::PoseStampedMessagePtr message_in_);
    static geometry_msgs::PoseWithCovarianceStampedPtr
    convert(const srrg2_core::PoseWithCovarianceStampedMessagePtr message_in_);
    static nav_msgs::PathPtr convert(const srrg2_core::PathMessagePtr message_in_);
    static geometry_msgs::TwistPtr convert(const srrg2_core::CmdVelMessagePtr message_in_);
    static grid_map_msgs::GridMapPtr convert(const srrg2_core::GridMapMessagePtr message_in_);
    static srrg2_core_ros::PlannerStatusMessagePtr
    convert(srrg2_core::PlannerStatusMessagePtr message_in_);

    // static geometry_msgs::TransformStampedPtr convert(const srrg2_core::TransformEvent*
    // message_in_); TODO used?

    // ds ROS to SRRG wrappers
  public:
    // ds templated conversion for ROS bag messages (in case an inner or outer type has to be
    // specified)
    template <typename ROSMessageInstanceType_>
    static srrg2_core::BaseSensorMessagePtr convert(const rosbag::MessageInstance& message_in_) {
      return Converter::convert(message_in_.instantiate<ROSMessageInstanceType_>());
    }

    // ds templated conversion to specific output type (wrapping the dynamic_pointer_cast)
    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::CameraInfoConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::ImageConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::ImuConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::RangeConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::LaserScanConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(nav_msgs::OdometryConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_>
    convert(geometry_msgs::PointStampedConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(tf2_msgs::TFMessageConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_>
    convert(geometry_msgs::TwistStampedConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::PointCloud2ConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::JointStateConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(sensor_msgs::NavSatFixConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_> convert(geometry_msgs::PoseArrayConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_>
    convert(geometry_msgs::PoseWithCovarianceStampedConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    template <typename SRRGMessageType_>
    static std::shared_ptr<SRRGMessageType_>
    convert(geometry_msgs::PoseStampedConstPtr message_in_) {
      return std::dynamic_pointer_cast<SRRGMessageType_>(convert(message_in_));
    }

    // ds SRRG to ROS wrappers
  public:
    // ds templated conversion from SRRG base type
    template <typename ROSMessageType_>
    static boost::shared_ptr<ROSMessageType_>
    convert(const srrg2_core::BaseSensorMessagePtr message_in_) {
      boost::shared_ptr<ROSMessageType_> message_out = boost::make_shared<ROSMessageType_>();
      return message_out;
    }

    // ds helpers
  protected:
    // srrg helpers
    static srrg2_core::TransformEvent _convert(const geometry_msgs::TransformStamped& message_in_);
    static srrg2_core::TransformEvent
    _convert(const geometry_msgs::TransformStampedConstPtr& message_in_);
  };
} // namespace srrg2_core_ros
