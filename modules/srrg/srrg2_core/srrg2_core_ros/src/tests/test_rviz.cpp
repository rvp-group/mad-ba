#include <iostream>
#include <ros/ros.h>
#include <srrg_system_utils/shell_colors.h>
#include <visualization_msgs/Marker.h>

#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/viewer_manager_shared.h>

#include <GL/glut.h>
#include <signal.h>
#include <thread>

#define NODE_NAME "node_jesus"
#define PUB_NAME "publisher_jesus"
#define PUB_QUEUE_SIZE 1
#define ROS_MAX_FREQ 2 // hz
#define POINTS_BULK_SIZE 2000
#define WORLD_SIZE 100
#define MAX_NUM_STEPS 10

using namespace srrg2_core;

void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    std::cerr << FG_RED("shutting down") << std::endl;
    ros::shutdown();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  ros::Rate ros_rate(ROS_MAX_FREQ);
  ros::Publisher publisher_marker = nh.advertise<visualization_msgs::Marker>(
    PUB_NAME, PUB_QUEUE_SIZE); // ia publisher name, queue size

  signal(SIGINT, sigIntHandler);

  // ia create some points
  PointNormal3fVectorCloud pointcloud;
  pointcloud.resize(POINTS_BULK_SIZE);
  for (auto& p : pointcloud) {
    p.coordinates() = Vector3f::Random() * WORLD_SIZE;
    p.normal()      = Vector3f::UnitX();
  }

  //  size_t cnt = 0;
  while (ros::ok() /*&& cnt++ < MAX_NUM_STEPS*/) {
    // ia create a marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp    = ros::Time::now();

    marker.type   = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points.resize(pointcloud.size());

    for (size_t i = 0; i < pointcloud.size(); ++i) {
      marker.points[i].x = pointcloud[i].coordinates().x();
      marker.points[i].y = pointcloud[i].coordinates().y();
      marker.points[i].z = pointcloud[i].coordinates().z();
    }

    //    marker.pose.position.x = 0;
    //    marker.pose.position.y = 0;
    //    marker.pose.position.z = 0;
    //    marker.pose.orientation.x = 0.0;
    //    marker.pose.orientation.y = 0.0;
    //    marker.pose.orientation.z = 0.0;
    //    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0f;
    marker.color.g = 0.2f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0f;

    // ia if noone is listening simpli ignore everything
    while (publisher_marker.getNumSubscribers() < 1) {
      if (!ros::ok())
        return 0;
      std::cerr << FG_YELLOW("no subscriber") << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // ia publish the marker
    publisher_marker.publish(marker);

    ros_rate.sleep(); // ia sleep for publisher
  }

  ros::shutdown();
  return 0;
}
