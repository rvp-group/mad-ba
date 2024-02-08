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

  void PointCloudProc::publishNormals()
  {
      Eigen::Isometry3d pose;
      pose = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY());  // rotate along X axis by 45 degrees
      static int x = 0;
      x++;
      pose.translation() = Eigen::Vector3d(0.1*x, 0.1*x , 0.1);           // translate x,y,z
      // Publish arrow vector of pose
      visual_tools_->publishArrow(pose, rviz_visual_tools::C_RED, rviz_visual_tools::XXLARGE);
      visual_tools_->trigger();
  }

  void PointCloudProc::createKDTree(std::shared_ptr<std::vector<Eigen::Vector3d>> curr_cloud) {

      // using ContainerType = std::vector<Eigen::Vector3d>;
      // using ContainerTypePtr = std::shared_ptr<ContainerType>;
      // using TreeNodeType = TreeNode3D<ContainerType>;
      // using TreeNodeTypePtr = TreeNodeType*;

      // TreeNodeTypePtr current_tree_ = new TreeNodeType(curr_cloud,
      //                                  curr_cloud->begin(),
      //                                  curr_cloud->end(),
      //                                  0.2,
      //                                  0.1,
      //                                  0,
      //                                  2,
      //                                  nullptr,
      //                                  nullptr);

      // std::vector<TreeNodeTypePtr> vect;
      // std::back_insert_iterator<std::vector<TreeNodeTypePtr>> it(vect);
      // // std::back_insert_iterator<std::vector<TreeNode3D*>> it;
      // current_tree_->getLeafs(it);
      // std::cout << "Vector size " << vect.size() << std::endl;
  } 

  bool PointCloudProc::putMessage(srrg2_core::BaseSensorMessagePtr msg) {

      // Convert message to srrg2 point cloud
      PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg);
      if (!cloud) {
          std::cerr << "PointCloudProc::putMessage | no msg received" << std::endl;
          return false;
      }

      // Print the general info for debugging
      static int cnt = 0;
      if (cnt >= 1)
          exit(0);
      std::cout << std::setprecision(12) << "Cnt: " << ++cnt << " TS: " << cloud->timestamp.value()
                << " Cloud height: " << cloud->height.value() << " Cloud width: " << cloud->width.value() << std::endl;


      // Change the frame_id of the point cloud
      cloud->frame_id.value() = "map";
      // Convert srrg2 to sensor_msgs point cloud
      sensor_msgs::PointCloud2ConstPtr rosMsg = Converter::convert(cloud);

      // Publish the raw cloud as ROS message
      pointCloudPub_.publish(*rosMsg);

      // Create Point3f point cloud
      Point3fVectorCloud cloudPoint3f;
      cloud->getPointCloud(cloudPoint3f);
      
      // Conver point cloud to the std::vector<Eigen::Vector3d
      std::vector<Eigen::Vector3d> pointCloudEigen;
      // TODO: optimize
      // For now copy all point from Point3f point cloud to std::vector<Eigen::Vector3d>
      for (const Point3f& p : cloudPoint3f) {
          Eigen::Vector3d pe(p.coordinates().cast<double>());
          pointCloudEigen.push_back(pe);
      }
      // For each point cloud create an KDTree
      
      using ContainerType = std::vector<Eigen::Vector3d>;
      // using ContainerTypePtr = std::shared_ptr<ContainerType>;
      using TreeNodeType = TreeNode3D<ContainerType>;
      using TreeNodeTypePtr = TreeNodeType*;

      TreeNodeTypePtr current_tree_ = new TreeNodeType(std::make_shared<ContainerType>(pointCloudEigen),
                                       pointCloudEigen.begin(),
                                       pointCloudEigen.end(),
                                       0.2,
                                       0.1,
                                       0,
                                       2,
                                       nullptr,
                                       nullptr);

      std::vector<TreeNodeTypePtr> vect;
      std::back_insert_iterator<std::vector<TreeNodeTypePtr>> it(vect);
      // std::back_insert_iterator<std::vector<TreeNode3D*>> it;
      current_tree_->getLeafs(it);
      std::cout << "Vector size " << vect.size() << std::endl;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      for (auto& el : vect) {
          // std::cout << el->eigenvectors_.col(0) << std::endl;
          pose.translation() = Eigen::Vector3d(el->mean_);  // translate x,y,z
          Eigen::Matrix3d rotMatrix = calculateMatrixBetween2Vectors(Eigen::Vector3d(1,0,0),el->eigenvectors_.col(0));
          pose.linear() = rotMatrix;
          visual_tools_->publishArrow(pose, rviz_visual_tools::C_RED, rviz_visual_tools::LARGE);
      }
      visual_tools_->trigger();

      // createKDTree(cloud_vect);
      // publishNormals();

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
