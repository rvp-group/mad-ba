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
      pointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw_point_cloud", 100);
      synthPointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("synth_point_cloud", 100);
      odomPub_ = nh_.advertise<nav_msgs::Odometry>("ba_estimate", 100);
      poseArrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("surfel_pose_array", 100);

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
      
      // Whether to use synthetic data
      bool useSynthethicData = false;

      // Skip first n messages and process only m first clouds
      uint cloudsToSkip = 100;
      uint cloudsToProcess = 2;
      static int msgCnt = -1;
      if (++msgCnt < 3 * cloudsToSkip)
          return true;
      else if (msgCnt > 3 * (cloudsToSkip + cloudsToProcess)) {
          // Publish results before exit
          // publishCloudNormals(0);                         // First normals
          // publishCloudNormals(kdTreeLeafes_.size() - 1);  // Last normals
          mergeSurfels();
          visualizeCorrespondingSurfelsWithPoses();
          ros::Duration(2.0).sleep();
          exit(0);
      }

      // Replace real data with synthetic one
      if (useSynthethicData) {
          sensor_msgs::PointCloud2 cloudMsgSynthetic;
          generateSyntheticPointCloud(cloudMsgSynthetic);

          // Replace real cloud with synthetic one
        //   sensor_msgs::PointCloud2ConstPtr rosMsg = Converter::convert(cloud);
        //   ros::Duration(2.0).sleep();
        //   exit(0);
      } else {
          // Let's assume that for each point cloud in a .bag file there is one odometry pose
          // Try to convert the message to srrg2 point cloud
          if (PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg)) {
              static int cloudMsgCnt = 0;
              std::cout << std::setprecision(12) << "Point cloud message no." << cloudMsgCnt++ << " Ts: " << cloud->timestamp.value() << std::endl;
              handleCloudMessage(cloud);
          }
          // Try to convert the message to srrg2 odometry
          else if (OdometryMessagePtr odom = std::dynamic_pointer_cast<OdometryMessage>(msg)) {
              static int odomMsgCnt = 0;
              std::cout << std::setprecision(12) << "Odometry message no." << odomMsgCnt++ << " Ts: " << odom->timestamp.value() << std::endl;
              handleOdometryMessage(odom);
          }
          // Try to convert the message to srrg2 odometry
          else if (TransformEventsMessagePtr tfMsg = std::dynamic_pointer_cast<TransformEventsMessage>(msg)) {
              static int tfMsgCnt = 0;
              std::cout << std::setprecision(12) << "Transform message no." << tfMsgCnt++ << " Ts: " << tfMsg->timestamp.value() << std::endl;
              handleTFMessage(tfMsg);
          }
          // Other messages types
          else {
              std::cerr << "PointCloudProc::putMessage | no handler for the received message type " << std::endl;
              return false;
          }
      }

      return true;
  }

  void PointCloudProc::visualizeSurfel(TreeNodeType* surfel, int markerColor) {
      Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
      surfelPose.translation() = Eigen::Vector3d(surfel->mean_);
      // Works even for surfels that have only 1 point (they have only the first column of egigenvector matrix)
      surfelPose.linear() = matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), surfel->eigenvectors_.col(0));
      // Get the surfel bounding box
      Eigen::Vector3d bbox = surfel->bbox_;
      for (int i = 0; i < 3; i++)
          if (bbox[i] < 0.01)
              bbox[i] = 0.01;

        // Publish normal as Arrow
      visual_tools_->publishArrow(surfelPose, static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::SMALL);

        // Publish surfel as Cylinder
        // Get the rotation matrix for 90* in Y axis
        Eigen::Matrix3d rotationY;
        rotationY << 0, 0, 1, 0, 1, 0, -1, 0, 0;
        surfelPose.linear() = surfelPose.linear() * rotationY;
        double radius = (bbox[1] + bbox[2]) / 2.0;
        visual_tools_->publishCylinder(surfelPose, static_cast<rviz_visual_tools::colors>(markerColor), bbox[0], radius);
        
        // Check if eigenvectors contains NaNs
        if (surfel->eigenvectors_ == surfel->eigenvectors_) {
            Eigen::Vector3d deltaX = surfel->eigenvectors_.col(0) * bbox[0];
            Eigen::Vector3d deltaY = surfel->eigenvectors_.col(1) * bbox[1];
            Eigen::Vector3d deltaZ = surfel->eigenvectors_.col(2) * bbox[2];

            // Publish the bound box diagonals
            visual_tools_->publishLine(surfel->mean_ - deltaX / 2.0, surfel->mean_ + deltaX / 2.0, static_cast<rviz_visual_tools::colors>(markerColor % 14));
            visual_tools_->publishLine(surfel->mean_ - deltaY / 2.0, surfel->mean_ + deltaY / 2.0, static_cast<rviz_visual_tools::colors>(markerColor % 14));
            visual_tools_->publishLine(surfel->mean_ - deltaZ / 2.0, surfel->mean_ + deltaZ / 2.0, static_cast<rviz_visual_tools::colors>(markerColor % 14));

            // Publish surfel as rectangle
            surfelPose.linear() = surfel->eigenvectors_ * rotationY;
            visual_tools_->publishWireframeRectangle(surfelPose, bbox[1], bbox[2], static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::XLARGE);
        }
      visual_tools_->trigger();
  }

  void PointCloudProc::visializeAllSurfels() {
      Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
      uint8_t markerColor = 0;
      long surfelCnt = 0; 
      int decimation = 10;

      //  Get all the surfels possible
      for (auto treeLeafs : kdTreeLeafes_) {
          // Change color
          if (++markerColor > 14)
              markerColor = 0;

          // Get the surfels
          for (auto& surfel : treeLeafs) {
              // Decimate for Rviz
              if (surfelCnt++ % decimation != 0)
                  continue;
              // Set the translation part
              surfelPose.translation() = Eigen::Vector3d(surfel->mean_);
              // Calculate the rotation matrix
              Eigen::Matrix3d rotMatrix = matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), surfel->eigenvectors_.col(0));
              surfelPose.linear() = rotMatrix;
              // Publish normal as arrow
              visual_tools_->publishArrow(surfelPose, static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::XXXLARGE);
          }
      }
      visual_tools_->trigger();
  }

  void PointCloudProc::visualizeSurfelPoses() {
      if (surfels_.size() == 0)
          return;
      // Take the first surfel
      for (const std::shared_ptr<Surfel>& surfel : surfels_) {
        //   if (surfel->poses_.size() < 5)
        //       continue;
          // Create pose array message
          geometry_msgs::PoseArray poseArray;
          poseArray.header.stamp = ros::Time::now();
          poseArray.header.frame_id = "map";

          // Add all poses for given surfel
          for (const Eigen::Isometry3d& pose : surfel->poses_) {
              // Create new pose Msg
              geometry_msgs::Pose poseMsg;
              poseMsg.position.x = pose.translation().x();
              poseMsg.position.y = pose.translation().y();
              poseMsg.position.z = pose.translation().z();
              // Add orientation
              Eigen::Quaterniond quat(pose.linear());
              poseMsg.orientation.x = quat.x();
              poseMsg.orientation.y = quat.y();
              poseMsg.orientation.z = quat.z();
              poseMsg.orientation.w = quat.w();
              // Add pose to poseArray
              poseArray.poses.push_back(poseMsg);
            //   std::cout << "Poses " << pose.matrix() << std::endl;
          }
          poseArrayPub_.publish(poseArray);
      }
  }

  void PointCloudProc::visualizeCorrespondingSurfelsWithPoses() {

      unsigned int maxPoses = 0, idx = 0;
      for (unsigned int i = 0; i < surfels_.size(); i++) {
          if (surfels_.at(i)->poses_.size() > maxPoses) {
              maxPoses = surfels_.at(i)->poses_.size();
              idx = i;
          }
      }

      // Add all poses for given surfel
      geometry_msgs::PoseArray poseArray;
      poseArray.header.stamp = ros::Time::now();
      poseArray.header.frame_id = "map";
      for (unsigned int i = 0; i < surfels_.size(); i++) {
          for (const Eigen::Isometry3d& pose : surfels_.at(i)->poses_) {
              // Create new pose Msg
              geometry_msgs::Pose poseMsg;
              poseMsg.position.x = pose.translation().x();
              poseMsg.position.y = pose.translation().y();
              poseMsg.position.z = pose.translation().z();
              // Add orientation
              Eigen::Quaterniond quat(pose.linear());
              poseMsg.orientation.x = quat.x();
              poseMsg.orientation.y = quat.y();
              poseMsg.orientation.z = quat.z();
              poseMsg.orientation.w = quat.w();
              // Add pose to poseArray
              poseArray.poses.push_back(poseMsg);
          }
          // Publish poses array
          poseArrayPub_.publish(poseArray);
          static int markerColor = 4;
          markerColor++;
          // Show all correspondences in one color
          for (auto const& poseId : surfels_.at(i)->posesIds_) {
              //   Visuzalize surfels with poses num > maxPoses/2
              //   if (surfels_.at(i)->poses_.size() == 3)
              {
                  static int visCnt = 0;
                  if (visCnt++ > 500)
                      break;
                  for (auto const& surfelId : poseId.second) {
                      TreeNodeType* leaf = kdTreeLeafes_[poseId.first].at(surfelId);
                      visualizeSurfel(leaf, markerColor % 14);

                      // Add line between pose and surfel
                      Eigen::Isometry3d trans = poses_.at(poseId.first);
                      visual_tools_->publishLine(trans.translation(), leaf->mean_, static_cast<rviz_visual_tools::colors>(markerColor % 14));
                  }
              }
          }
      }
  }

  void PointCloudProc::saveSurfelsTofile() {
      // Sort the surfels based on number of surfels its created from
      std::sort(surfels_.begin(), surfels_.end(), [](const std::shared_ptr<Surfel>& a, const std::shared_ptr<Surfel>& b) {
          return a->poses_.size() < b->poses_.size();
      });
      // Add all surfels as json object
      auto jsonArray = nlohmann::json::array();
      for(unsigned int i = 0; i < surfels_.size(); ++i) {
          jsonArray.push_back(surfels_.at(i)->getJson());
      }
      // Save to file
      std::ofstream o("pretty.json");
      o << std::setw(4) << jsonArray.dump() << std::endl;
  }

  void PointCloudProc::mergeSurfels() {

      // Parameters for merging
      // Max distance between two surfels to consider the merge
      double maxDistance = 0.05;
      // Max angle between two surfels to consider the merge
      double maxAngle = 2.0 * M_PI / 180.0;

    //   Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
      u_long markerColor = 0;
    //   long surfelCnt = 0; 
    //   int decimation = 10;

      int matchedSurfels = 0;
      // Iterate over all kdTrees (poses) pairs
      for (unsigned long i = 0; i < kdTreeLeafes_.size() - 1; i++) {
          for (unsigned long j = i + 1; j < kdTreeLeafes_.size(); j++) {
              // For each surfer from i-th kdTree find the closest surfer from j-th kdTree
            //   for (auto& leafI : kdTreeLeafes_[i]) {
              for (unsigned int idLeafI = 0; idLeafI < kdTreeLeafes_[i].size(); idLeafI++) {
                  //   auto kdTreetmpp = kdTrees_.at(j);
                  // std::shared_ptr<TreeNodeType> surfelB = std::shared_ptr<TreeNodeType>(kdTreetmpp->bestMatchingLeafFast(surfelA->mean_));
                  // Get pointer to a surfelB
                  TreeNodeType* leafI = kdTreeLeafes_[i].at(idLeafI);
                  TreeNodeType* leafJ = kdTrees_.at(j)->bestMatchingLeafFast(leafI->mean_);
                //   std::cout << "Mean of the leaf:  "<< leafI->mean_ << std::endl;
                  //   std::cout << "SurfelB size:  "  << surfelB->num_points_ << std::endl;
                  //   auto& surfelB = (kdTrees_.at(j))->bestMatchingLeafFast(surfelA->mean_);
                  double distance = (leafJ->mean_ - leafI->mean_).norm();
                  if (distance < maxDistance) {
                      double angle = angleBetween2Vectors(leafI->eigenvectors_.col(0), leafJ->eigenvectors_.col(0));
                      if (angle < maxAngle) {
                          // Visualize the surfels for merge
                        //   visualizeSurfel(leafI, markerColor % 14);
                        //   visualizeSurfel(leafJ, markerColor % 14);
                        //   markerColor++;
                          matchedSurfels++;

                          // Need to find the leafJid in the vector anyway:
                          // ToDo optimize somehow because its very slow - give ids to surfels or sth
                          int idTmp = findLeafId(j, leafJ);
                          if (idTmp == -1){
                              std::cerr << "Not found id of existing leaf" << std::endl;
                              exit(0);
                          }
                          unsigned int idLeafJ = (unsigned int) idTmp;
                          
                          // If correspondence found then check if that surfel already exists
                          std::shared_ptr<Surfel> surfel;
                          bool foundSurfelI = false, foundSurfelJ = false;
                          for (std::shared_ptr<Surfel> tmpSurfel : surfels_) {
                              if (tmpSurfel->checkIfsurfelIdExists(i, idLeafI)) {
                                  surfel = tmpSurfel;
                                  foundSurfelI = true;
                                  break;
                              }
                              if (tmpSurfel->checkIfsurfelIdExists(j, idLeafJ)) {
                                  surfel = tmpSurfel;
                                  foundSurfelJ = true;
                                  break;
                              }
                          }

                          // If surfel doesnt exists then create it
                          if (!surfel) {
                              surfel = std::make_shared<Surfel>();
                          }

                          // Create observation for given surfel from pose I
                          if (!foundSurfelI) {
                              Eigen::Matrix<double, 9, 1> observationI = Eigen::Matrix<double, 9, 1>::Identity();
                              observationI.block<3, 1>(0, 0) = leafI->mean_;
                              observationI.block<3, 1>(3, 0) = leafI->eigenvectors_.col(0).transpose();
                              observationI.block<3, 1>(6, 0) = leafI->bbox_;
                              surfel->addObservation(poses_.at(i), i, observationI);
                          }

                          // Create observation for given surfel from pose J
                          if (!foundSurfelJ) {
                              Eigen::Matrix<double, 9, 1> observationJ = Eigen::Matrix<double, 9, 1>::Identity();
                              observationJ.block<3, 1>(0, 0) = leafI->mean_;
                              observationJ.block<3, 1>(3, 0) = leafI->eigenvectors_.col(0).transpose();
                              observationJ.block<3, 1>(6, 0) = leafI->bbox_;
                              surfel->addObservation(poses_.at(j), j, observationJ);
                          }

                          // ToDo: make Surfel method for this
                          // Add leafI in a kdTreeI
                          surfel->posesIds_[i].insert(idLeafI);
                          // Add leafJ in a kdTreeJ
                          surfel->posesIds_[j].insert(idLeafJ);
                          // Add surfel to a vector for storage
                          if (foundSurfelI == false && foundSurfelJ == false)
                              surfels_.push_back(std::move(surfel));
                      }
                  }
              }
              visual_tools_->trigger();
              std::cout << " i: " << i << "  j:  " << j << std::endl;
              std::cout << " Matched surfels "  << matchedSurfels << std::endl;
          }
      }
  }

  int PointCloudProc::findLeafId(unsigned int kdTreeId, TreeNodeTypePtr leaf) {
      for (unsigned int i = 0; i < kdTreeLeafes_[kdTreeId].size(); i++) {
          if (leaf->mean_ == kdTreeLeafes_[kdTreeId].at(i)->mean_)
              return i;
      }
      return -1;
  }

  void PointCloudProc::handleTFMessage(TransformEventsMessagePtr tfMsg) {
      // Just republish the message to ROS
      tf2_msgs::TFMessageConstPtr tfMsgPtr = Converter::convert(tfMsg);
      for (geometry_msgs::TransformStamped tfStamped : (*tfMsgPtr).transforms) {
          transformBroadcaster_.sendTransform(tfStamped);
      }
  }

  void PointCloudProc::handleOdometryMessage(OdometryMessagePtr odom) {
      // Just republish the message to ROS
      nav_msgs::OdometryConstPtr rosMsgPtr = Converter::convert(odom);
      odomPub_.publish(*rosMsgPtr);
      // Save the pose in a vector
      Eigen::Vector3d trans(rosMsgPtr->pose.pose.position.x, rosMsgPtr->pose.pose.position.y, rosMsgPtr->pose.pose.position.z);
      Eigen::Quaterniond quat(rosMsgPtr->pose.pose.orientation.w, rosMsgPtr->pose.pose.orientation.x, rosMsgPtr->pose.pose.orientation.y, rosMsgPtr->pose.pose.orientation.z);
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = trans;
      pose.linear() = quat.toRotationMatrix();
      poses_.push_back(pose);
  }

  void PointCloudProc::handleCloudMessage(PointCloud2MessagePtr cloudMsg) {

      // Create buffer for messages
      static std::vector<PointCloud2MessagePtr> cloudBuffer;
      // Add item to a buffer
      cloudBuffer.push_back(cloudMsg); 
      
      // Try to empty the buffer
      for (unsigned int i = 0; i < cloudBuffer.size(); i++) {

          // Check if there is already pose for that cloud
          // Get the index that this cloud will have
          int idx_cloud = pointClouds_.size();
          int idx_pose = poses_.size() - 1;

          // If there is no pose for the cloud then add it to a buffer
          if (idx_cloud > idx_pose) {
            //   cloudBuffer.push_back(std::shared_ptr(cloud));
              std::cout << "No pose for the pointcloud   |  Buffer size:  " << cloudBuffer.size() << std::endl;
              return;
          }

          // Get the reference to first item in buffer
          PointCloud2MessagePtr & cloud = cloudBuffer.at(i);

          // Change the frame_id of the point cloud
          cloud->frame_id.value() = "ba_estimate";
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

          // For each point cloud create an KDTree and leafs | Get the last pose 
          createKDTree(pointCloudEigen, poses_.back());
          // Publish the normals created from the last point cloud
          //   int idx = kdTreeLeafes_.size()-1;
          //   publishCloudNormals(idx);

          // Store Point3f cloud in a vector
          pointClouds_.push_back(std::move(pointCloudPoint3f));

          // Remove cloud from buffer
          cloudBuffer.erase(cloudBuffer.begin());
      }
  }

  Eigen::Matrix3d PointCloudProc::matrixBetween2Vectors(Eigen::Vector3d a, Eigen::Vector3d b) {
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

  double PointCloudProc::angleBetween2Vectors(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
    double angle = atan2(a.cross(b).norm(), a.dot(b));
    // if (angle > M_PI / 2.0)
    //     return M_PI - angle;
    // else
    return angle;
  }

  void PointCloudProc::publishCloudNormals(int idx) {
      // Delete all markers from Rviz
        // visual_tools_->deleteAllMarkers();

      static uint8_t markerColor = 0;  // Colors change <0,14>
      if (++markerColor > 14)
          markerColor = 0;

      uint8_t decimation = 10;

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      for (auto& leaf : kdTreeLeafes_.at(idx)) {
          // Decimate for Rviz
          if (static int cnt = 0; cnt++ % decimation != 0)
              continue;
          // Set the translation part
          pose.translation() = Eigen::Vector3d(leaf->mean_);
          // Calculate the rotation matrix
          Eigen::Matrix3d rotMatrix = matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), leaf->eigenvectors_.col(0));
          pose.linear() = rotMatrix;
          // Publish normal as arrow
          visual_tools_->publishArrow(pose, static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::XXXLARGE);
      }

      // Trigger publishing of all arrows
      visual_tools_->trigger();
  }

  void PointCloudProc::createKDTree(std::vector<Eigen::Vector3d>& cloud, const Eigen::Isometry3d& lastPose) {
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

      // Transform kdTree according to the BA pose
      // Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
      // Eigen::Vector3d trans = Eigen::Vector3d(20,20,20);
      kdTree->applyTransform(lastPose.linear(), lastPose.translation());
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

  void PointCloudProc::generateSyntheticPointCloud(sensor_msgs::PointCloud2 & cloudMsg) {
      
      static std::random_device rd;                           // obtain a random number from hardware
      static std::mt19937 gen(rd());                          // seed the generator
      static std::uniform_int_distribution<> distrX(1, 300);  //  // define the range
      static std::uniform_int_distribution<> distrY(1, 200);  //  // define the range
      static std::uniform_int_distribution<> distrZ(1, 20);   //  // define the range

      // Synthethic cloud
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pointCloudSynth;

      // Room dimensons
      double width = 10, depth = 15, height = 5;
      double pointsDensity = 10;  // Pts per meter
      Eigen::Vector3d point;
      // Generate floor
      for (int i = 0; i <= width * pointsDensity; i++) {
          for (int j = 0; j <= depth * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3d(i / pointsDensity, j / pointsDensity, 0));
          }
      }
      // Generate front and back walls
      for (int i = 0; i <= width * pointsDensity; i++) {
          for (int j = 1; j < height * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3d(i / pointsDensity, 0, j / pointsDensity));
              pointCloudSynth.push_back(Eigen::Vector3d(i / pointsDensity, depth, j / pointsDensity));
          }
      }

      // Generate left and right walls
      for (int i = 1; i < depth * pointsDensity; i++) {
          for (int j = 1; j < height * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3d(0, i / pointsDensity, j / pointsDensity));
              pointCloudSynth.push_back(Eigen::Vector3d(width, i / pointsDensity, j / pointsDensity));
          }
      }

      cloudMsg.header.frame_id = "map";
      cloudMsg.header.stamp = ros::Time::now();
      cloudMsg.height = 1;
      cloudMsg.width  = pointCloudSynth.size();
      std::cout << "Num of points:  " << pointCloudSynth.size()  << "  Synth Cloud size:  "  << cloudMsg.height * cloudMsg.width << std::endl;
      
      //   sensor_msgs::PointField fieldX, fieldY,  fieldZ;
      //   fieldX.name = "x";
      //   fieldX.offset = 0;
      //   fieldX.datatype = 7;
      //   fieldX.count = 1;
      //   fieldY.name = "y";
      //   fieldY.offset = 4;
      //   fieldY.datatype = 7;
      //   fieldY.count = 1;
      //   fieldZ.name = "z";
      //   fieldZ.offset = 8;
      //   fieldZ.datatype = 7;
      //   fieldZ.count = 1;
      //   cloudMsg.fields.push_back(fieldX);
      //   cloudMsg.fields.push_back(fieldY);
      //   cloudMsg.fields.push_back(fieldZ);
      cloudMsg.is_bigendian = false;
      //   cloudMsg.point_step = 12;
      //   cloudMsg.row_step = cloudMsg.width * cloudMsg.point_step;
      cloudMsg.is_dense = true;
      // Create the modifier to setup the fields and memory
      sensor_msgs::PointCloud2Modifier cloudModifier(cloudMsg);
      // Set the fields that our cloud will have
      cloudModifier.setPointCloud2FieldsByString(1, "xyz");

      // Create iterators
      sensor_msgs::PointCloud2Iterator<float> iterX(cloudMsg, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(cloudMsg, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(cloudMsg, "z");
      for (auto& point : pointCloudSynth) {
          //   for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
          if (iterX != iterX.end()) {
              Eigen::Vector3f point3f = point.cast<float>();
              *iterX = point3f[0];
              *iterY = point3f[1];
              *iterZ = point3f[2];
              ++iterX;
              ++iterY;
              ++iterZ;
          } else
              break;
      }
      synthPointCloudPub_.publish(cloudMsg);

      //   for (int i = 0; i < 200; i++) {
      //       Eigen::Vector3d point = Eigen::Vector3d(distrX(gen) / 1e3, distrY(gen) / 1e3, distrZ(gen) / 1e3);
      //       // transform the point
      //     //   point = poses_.back() * point;
      //       pointCloudSynth.push_back(point);
      //   }
      visual_tools_->publishSpheres(pointCloudSynth, static_cast<rviz_visual_tools::colors>(9), rviz_visual_tools::SMALL);
      visual_tools_->trigger();
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
