#include "point_cloud_proc.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_pcl/point_normal_curvature.h>
#include <srrg_pcl/instances.h>
#include <srrg_converters/converter.h>
#include "kdtree.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

namespace structure_refinement {
  using namespace srrg2_core;
  using namespace srrg2_core_ros;
  using LidarProjectorType   = PointIntensity3fProjectorOS1_64;
  using LidarUnprojectorType = PointIntensity3fUnprojectorOS1_64;
  using NormalComputatorType = NormalComputator2DCrossProduct<PointNormal3fVectorCloud,0>; //NormalComputator2DCrossProduct<PointNormal3fMatrixCloud, 1>;

  PointCloudProc::PointCloudProc() : nh_("~") {

      // ROS publishers
      pointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw_point_cloud", 1000);

      // Initalize rviz visual tools
      visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "rviz_visual_markers"));
      visual_tools_->loadMarkerPub();
      visual_tools_->deleteAllMarkers();
      visual_tools_->enableBatchPublishing();

      // Publish some tf for Rviz visualization
      static tf2_ros::StaticTransformBroadcaster staticBrd;
      geometry_msgs::TransformStamped static_transformStamped;
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "world";
      static_transformStamped.child_frame_id = "map";
      static_transformStamped.transform.rotation.w = 1.0;
      staticBrd.sendTransform(static_transformStamped);
  
  }

  PointCloudProc::~PointCloudProc()  {
  }

  bool PointCloudProc::putMessage(srrg2_core::BaseSensorMessagePtr msg) {

      // Convert message to srrg2 point cloud
      PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg);
      if (!cloud) {
          std::cerr << "PointCloudProc::putMessage | no msg received" << std::endl;
          return false;
      }

      // Print the general info for debugging
      static int msgCnt = -1;
      msgCnt++;
      if (msgCnt >= 10)
          exit(0);

      std::cout << std::setprecision(12) << "Cnt: " << msgCnt << " TS: " << cloud->timestamp.value()
                << " Cloud height: " << cloud->height.value() << " Cloud width: " << cloud->width.value() << std::endl;

      // Change the frame_id of the point cloud
      cloud->frame_id.value() = "map";
      // Convert srrg2 to sensor_msgs point cloud
      sensor_msgs::PointCloud2ConstPtr rosMsg = Converter::convert(cloud);

      // Publish the raw cloud as ROS message
      pointCloudPub_.publish(*rosMsg);

      // Create Point3f Point Cloud from srrg2 point cloud
      std::shared_ptr<Point3fVectorCloud> pointCloudPoint3f(new Point3fVectorCloud());      
      cloud->getPointCloud(*pointCloudPoint3f);

      // TODO: optimize
      // Conver point cloud to the std::vector<Eigen::Vector3d
      std::vector<Eigen::Vector3d> pointCloudEigen;
      // For now copy all the points from Point3f point cloud to std::vector<Eigen::Vector3d>
      for (const Point3f& p : *pointCloudPoint3f) {
          Eigen::Vector3d pe(p.coordinates().cast<double>());
          pointCloudEigen.push_back(pe);
      }

      // For each point cloud create an KDTree
      createKDTree(pointCloudEigen);
      publishNormals(msgCnt);

      // Store Point3f cloud in a vector
      pointClouds_.push_back(std::move(pointCloudPoint3f));
      return true;
  }

  Eigen::Matrix3d PointCloudProc::calculateMatrixBetween2Vectors(Eigen::Vector3d a, Eigen::Vector3d b) {
      a = a / a.norm();
      b = b / b.norm();
      Eigen::Vector3d v = a.cross(b);
      float s = v.norm();
      float c = a.dot(b);
      Eigen::Matrix3d vx;
      vx << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
      Eigen::Matrix3d r = Eigen::Matrix3d::Identity(3, 3);
      if (s != 0) {
          r = r + vx + vx * vx * ((1 - c) / std::pow(s, 2));
      } else {
          std::cout << "doesn't work if a == -b" << std::endl;
      }
      return r;
  }

  void PointCloudProc::publishNormals(int idx)
  {
      static uint8_t markerColor = 0;  // Colors change <0,14>
      if (++markerColor > 14)
          markerColor = 0;

    //   visual_tools_->deleteAllMarkers();

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      for (auto& leaf : kdTreeLeafes_.at(idx)) {
          // Set the translation part
          pose.translation() = Eigen::Vector3d(leaf->mean_);
          // Calculate the rotation matrix
          Eigen::Matrix3d rotMatrix = calculateMatrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), leaf->eigenvectors_.col(0));
          pose.linear() = rotMatrix;
          // Publish normal as arrow
          visual_tools_->publishArrow(pose, static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::MEDIUM);
      }

      // Trigger publishing of all arrows
      visual_tools_->trigger();
  }


  void PointCloudProc::createKDTree(std::vector<Eigen::Vector3d> &cloud) {

      using ContainerType = std::vector<Eigen::Vector3d>;
      using TreeNodeType = TreeNode3D<ContainerType>;
      using TreeNodeTypePtr = TreeNodeType*;

      // Create kdTree from the point cloud
      std::shared_ptr<TreeNodeType> kdTree(new TreeNodeType(cloud.begin(),
                                              cloud.end(),
                                              0.2,
                                              0.1,
                                              0,
                                              2,
                                              nullptr,
                                              nullptr));

      // Create a vector of leafes
      std::vector<TreeNodeTypePtr> leafes;
      // Create iterator for the leafes
      std::back_insert_iterator<std::vector<TreeNodeTypePtr>> it(leafes);
      // Get the leafes
      kdTree->getLeafs(it);
      // Pass leafes to a vector for storage
      kdTreeLeafes_.push_back(std::move(leafes));
      // Pass kdTree to a vector for storage
      kdTrees_.push_back(std::move(kdTree));
  } 


  // Just for test
  bool PointCloudProc::createIntensityImage(srrg2_core::BaseSensorMessagePtr msg) {
    PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg);
    if (!cloud) {
      std::cerr << "PointCloudProc::putMessage | no msg received" << std::endl;
      return false;
    }
    static int cnt = 0;
    cnt++;
    std::cerr << "Cnt: " << cnt << " Cloud height: " << cloud->height.value() << " Cloud width: " << cloud->width.value() << std::endl;
    PointIntensity3fVectorCloud cloud2;
    cloud->getPointCloud(cloud2);
    // cloud->PointNormalIntensity3fVectorCloud();

    LidarProjectorType projector;
    projector.param_num_columns.setValue(1024);
    projector.param_range_min.setValue(0.5);
    projector.param_range_max.setValue(100.0);
    projector.param_horizontal_start_angle.setValue(M_PI);
    projector.param_horizontal_end_angle.setValue(-M_PI);

    LidarProjectorType::TargetMatrixType target_matrix_type;
    LidarProjectorType::Lidar3DSensorType lidar_sensor;
    srrg2_core::ImageFloat intensity_image;
    cv::Mat cv_intensity_image;

    PointIntensity3fVectorCloud current_cloud;
    cloud->getPointCloud(current_cloud);
    projector.compute(target_matrix_type, current_cloud.begin(), current_cloud.end());

    intensity_image.resize(lidar_sensor.verticalResolution(), projector.param_num_columns.value());
    auto intensity_image_it = intensity_image.begin();
    for (auto it = target_matrix_type.begin(); it != target_matrix_type.end();
         ++it, ++intensity_image_it)
    {
      *intensity_image_it = it->source_it->intensity();
    }
    intensity_image.toCv(cv_intensity_image);
    cv_intensity_image = cv_intensity_image / 255.0;

    cv::imshow("lidar_intensity", cv_intensity_image);
    // ToDo - comment when using srgg2_shell and uncomment when running from app_point_cloud_proc 
    cv::waitKey(1);

    return true;
  }

} // namespace structure_refinement
