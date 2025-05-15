// #include "point_cloud_proc.h"
#include <bits/stdc++.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_config/pipeline_runner.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/message_handlers/message_sorted_sink.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_normal_curvature.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <mad_ba/point_cloud_proc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <vector>

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_lidar3d_utils;
using namespace srrg2_core;
using namespace srrg2_solver;

using ContainerType = std::vector<Eigen::Vector3f>;
using TreeNodeType = TreeNode3D<ContainerType>;
using TreeNodeTypePtr = TreeNodeType*;

void buildKDTreeAndGetLeafs(std::vector<Eigen::Vector3f>&, std::vector<TreeNodeTypePtr>&);

int main(int argc, char** argv) {
  // Load point cloud from .pcd file
  pcl::PointCloud<pcl::PointSurfel> psCloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/kcwian/Desktop/tmp_map/mesh.ply", *cloud) == -1) {
    PCL_ERROR("Couldn't read file.\n");
    return -1;
  }

  // Copy all the points from Point3f point cloud to std::vector<Eigen::Vector3f>
  std::vector<Eigen::Vector3f> pointCloudEigen;
  for (const pcl::PointXYZ& p : *cloud) {
    Eigen::Vector3f pe(p.x, p.y, p.z);
    pointCloudEigen.push_back(pe);
  }

  // For each point cloud create an KDTree and leafs | Get the last pose
  std::vector<TreeNodeTypePtr> leafes;
  buildKDTreeAndGetLeafs(pointCloudEigen, leafes);

  // Create new point cloud based on the leafs
  pcl::PointCloud<pcl::PointSurfel>::Ptr cloudAfter(new pcl::PointCloud<pcl::PointSurfel>);
  for (const TreeNodeTypePtr& leaf : leafes) {
    pcl::PointSurfel ps;
    ps.x = leaf->mean_[0];
    ps.y = leaf->mean_[1];
    ps.z = leaf->mean_[2];
    ps.normal_x = leaf->eigenvectors_.col(0)[0];
    ps.normal_y = leaf->eigenvectors_.col(0)[1];
    ps.normal_z = leaf->eigenvectors_.col(0)[2];
    uint8_t r = 255, g = 0, b = 0, a = 255;
    ps.r = r;
    ps.g = g;
    ps.b = b;
    float radius = sqrt(leaf->bbox_[1] * leaf->bbox_[1] + leaf->bbox_[2] * leaf->bbox_[2]) / 2.0;
    ps.radius = radius;
    cloudAfter->push_back(ps);
  }

  // Save new point cloud to file
  static std::string path = "/home/kcwian/Desktop/tmp_map/";
  pcl::io::savePCDFile(path + "meshAfter.pcd", *cloudAfter);
}

void buildKDTreeAndGetLeafs(std::vector<Eigen::Vector3f>& cloud, std::vector<TreeNodeTypePtr>& leafes) {
  static int pointCloudId = -1;
  pointCloudId++;
  // Create kdTree from the point cloud
  std::unique_ptr<TreeNodeType> kdTree = std::make_unique<TreeNodeType>(cloud.begin(), cloud.end(), 0.2, 0.1, 0, 2, nullptr, nullptr, pointCloudId);

  // Create a vector of leafes
  std::back_insert_iterator<std::vector<TreeNodeTypePtr>> it(leafes);
  kdTree->getLeafs(it);
}