#include "srrg_messages/instances.h"
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_property/property_container.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/messages/point_stamped_message.h>
#include <srrg_messages/messages/twist_stamped_message.h>
#include <srrg_messages/messages/navsat_fix_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_messages/messages/pose_with_covariance_stamped_message.h>
#include <srrg_messages/messages/pose_array_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/cmd_vel_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

int main(int argc_, char** argv_) {
  messages_registerTypes();
  return srrg2_test::runTests(argc_, argv_);
}

TEST(PointCloud2Message, SerializeDeserialize_Point3f) {
  const std::string message_file_name = "test_messages.json";

  // srrg produce random pointcloud
  const size_t num_points = 5;
  Point3fVectorCloud in_point_cloud;
  in_point_cloud.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    Vector3f coords = Vector3f::Random() * 100;
    Point3f p;
    p.coordinates() = coords;
    in_point_cloud.emplace_back(p);
  }
  // srrg produce a message
  PointCloud2MessagePtr message(new PointCloud2Message("/scan", "/scan", 69, 0.1091));
  message->setPointCloud(in_point_cloud);

  // srrg serialize it
  MessageFileSink msg_file_sink;
  msg_file_sink.open(message_file_name);
  msg_file_sink.putMessage(message);
  msg_file_sink.close();

  std::cerr << "serialized message" << std::endl;
  for (size_t i = 0; i < num_points; ++i) {
    std::cerr << in_point_cloud[i].coordinates().transpose() << std::endl;
  }
  // srrg deserialize it
  MessageFileSource msg_file_source;
  msg_file_source.open(message_file_name);
  Point3fVectorCloud out_point_cloud;
  BaseSensorMessagePtr base_msg = msg_file_source.getMessage();
  if (PointCloud2MessagePtr msg = std::dynamic_pointer_cast<PointCloud2Message>(base_msg)) {
    msg->getPointCloud(out_point_cloud);
  }
  std::cerr << "deserialized message" << std::endl;
  ASSERT_EQ(in_point_cloud.size(), out_point_cloud.size());

  for (size_t i = 0; i < out_point_cloud.size(); ++i) {
    const Vector3f& in_coords  = in_point_cloud[i].coordinates();
    const Vector3f& out_coords = out_point_cloud[i].coordinates();
    std::cerr << out_coords.transpose() << std::endl;
    ASSERT_EQ_EIGEN(in_coords, out_coords)
  }
}

TEST(PointCloud2Message, SerializeDeserialize_PointIntensity3f) {
  const std::string message_file_name = "test_messages.json";

  // srrg produce random pointcloud
  const size_t num_points = 5;
  PointIntensity3fVectorCloud in_point_cloud;
  in_point_cloud.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    Vector3f coords = Vector3f::Random() * 100;
    PointIntensity3f p;
    p.coordinates() = coords;
    p.intensity()   = i + 0.5;
    in_point_cloud.emplace_back(p);
  }
  // srrg produce a message
  PointCloud2MessagePtr message(new PointCloud2Message("/scan", "/scan", 69, 0.1091));
  message->setPointCloud(in_point_cloud);

  // srrg serialize it
  MessageFileSink msg_file_sink;
  msg_file_sink.open(message_file_name);
  msg_file_sink.putMessage(message);
  msg_file_sink.close();

  std::cerr << std::fixed << std::setprecision(6);
  std::cerr << "serialized message" << std::endl;
  for (size_t i = 0; i < num_points; ++i) {
    std::cerr << "coords [ " << in_point_cloud[i].coordinates().transpose() << " ] -- intensity [ "
              << in_point_cloud[i].intensity() << " ]\n";
  }
  // srrg deserialize it
  MessageFileSource msg_file_source;
  msg_file_source.open(message_file_name);
  PointIntensity3fVectorCloud out_point_cloud;
  BaseSensorMessagePtr base_msg = msg_file_source.getMessage();
  if (PointCloud2MessagePtr msg = std::dynamic_pointer_cast<PointCloud2Message>(base_msg)) {
    msg->getPointCloud(out_point_cloud);
  }
  std::cerr << "deserialized message" << std::endl;
  ASSERT_EQ(in_point_cloud.size(), out_point_cloud.size());

  for (size_t i = 0; i < out_point_cloud.size(); ++i) {
    const Vector3f& in_coords  = in_point_cloud[i].coordinates();
    const Vector3f& out_coords = out_point_cloud[i].coordinates();
    std::cerr << "coords [ " << out_point_cloud[i].coordinates().transpose() << " ] -- intensity [ "
              << out_point_cloud[i].intensity() << " ]\n";
    ASSERT_EQ_EIGEN(in_coords, out_coords)
    ASSERT_EQ(in_point_cloud[i].intensity(), out_point_cloud[i].intensity());
  }
}
