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
#include <cmath>
#include <rviz/SendFilePath.h>
#include <srrg_system_utils/chrono.h>
#include "data_association.cuh"


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
      surfelPointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("surfel_point_cloud", 100);

      odomPub_ = nh_.advertise<nav_msgs::Odometry>("ba_estimate", 100);
      poseArrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("surfel_pose_array", 100);
      beforeOptimPoseArrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("before_optim_pose_array", 100);
      afterOptimPoseArrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("after_optim_pose_array", 100);

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

      clientRviz_ = nh_.serviceClient<rviz::SendFilePath>("/my_rviz/load_config_discarding_changes");
  }

  PointCloudProc::~PointCloudProc()  {
  }

  bool PointCloudProc::putMessage(srrg2_core::BaseSensorMessagePtr msg) {
      
      srrg2_core::Chrono ch1("putMessage", &_timings, false);
      // Whether to use synthetic data
      bool useSynthethicData = false;

      // Skip first n messages and process only m first clouds
      int cloudsToSkip = 100;
      int decimateRealData = 10;
      int cloudsToProcess = 30 * decimateRealData;
      static int msgCnt = -1;
      if (++msgCnt < 2 * cloudsToSkip) {
        return true;
      } else if (msgCnt > 2 * (cloudsToSkip + cloudsToProcess) - 1) {
        for (int i = 0; i < 3; i++) {
          DataAssociation dataAssociation;
          {
            // srrg2_core::Chrono chGP("prepareData GPU", &_timings, false);
            // dataAssociation.prepareData(kdTrees_, kdTreeLeafes_);
          }

          {
            srrg2_core::Chrono chGP2("prepareData CPU", &_timings, false);
            dataAssociation.prepareDataCPU(kdTrees_, kdTreeLeafes_);
          }
          handleFactorGraphAfterGPU(dataAssociation.getSurfels());
          // visualizeCorrespondingSurfelsV2WithPoses(dataAssociation.getSurfels());

          // Reset the leafs and surfels
          resetLeafsSurfelId();
          dataAssociation.resetSurfels();
        }

        ros::Duration(1.0).sleep();
        srrg2_core::Chrono::printReport(_timings);
        exit(0);
      }
      if (useSynthethicData) {
          // Simulate pose, cloud, pose, cloud messages
          static bool msgType = 0;
          msgType ^= 1;
          if (msgType) {
              static int odomMsgCnt = 0;
              std::cout << "Synthetic odometry msg no. " << odomMsgCnt++ << std::endl;
              nav_msgs::OdometryPtr odomMsgSynthPtr(new nav_msgs::Odometry());
              generateSyntheticOdometry(*odomMsgSynthPtr);
              // Pointless but convert it to srrg odom, so that it can be processed the same way and converted back
              OdometryMessagePtr odomSrrg = std::dynamic_pointer_cast<OdometryMessage>(Converter::convert(odomMsgSynthPtr));
              handleOdometryMessage(odomSrrg);

          } else {
              static int cloudMsgCnt = 0;
              std::cout << "Synthetic point cloud msg no. " << cloudMsgCnt++ << std::endl;
              sensor_msgs::PointCloud2Ptr cloudMsgSynthPtr(new sensor_msgs::PointCloud2());
              generateSyntheticPointCloud(*cloudMsgSynthPtr);
              PointCloud2MessagePtr cloudSrrg = std::dynamic_pointer_cast<PointCloud2Message>(Converter::convert(cloudMsgSynthPtr));
              addNoiseToLastPose();
              handleCloudMessage(cloudSrrg);
          }
      } else {
          // Let's assume that for each point cloud in a .bag file there is one odometry pose
          // Try to convert the message to srrg2 point cloud
          if (PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg)) {
              static int cloudMsgCnt = 0;
              if (cloudMsgCnt++ % decimateRealData == 0) {
                std::cout << std::setprecision(12) << "Point cloud message no." << cloudMsgCnt << " Ts: " << cloud->timestamp.value() << std::endl;
                handleCloudMessage(cloud);
              }
          }
          // Try to convert the message to srrg2 odometry
          else if (OdometryMessagePtr odom = std::dynamic_pointer_cast<OdometryMessage>(msg)) {
            static int odomMsgCnt = 0;
            if (odomMsgCnt++ % decimateRealData == 0) {
              std::cout << std::setprecision(12) << "Odometry message no." << odomMsgCnt << " Ts: " << odom->timestamp.value() << std::endl;
              handleOdometryMessage(odom);
              addNoiseToLastPose();
            }
          }
          // Try to convert the message to tf message
          else if (TransformEventsMessagePtr tfMsg = std::dynamic_pointer_cast<TransformEventsMessage>(msg)) {
              static int tfMsgCnt = 0;
              std::cout << std::setprecision(12) << "Transform message no." << tfMsgCnt++ << " Ts: " << tfMsg->timestamp.value() << std::endl;
              // Disabled - now the TF transform is published in handleOdometryMessage() which makes it simpler to use synthethic data
              // handleTFMessage(tfMsg);
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
      visual_tools_->publishArrow(surfelPose, static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::MEDIUM);

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

  void PointCloudProc::visializeSurfelsv2(std::vector<Surfelv2>& surfelsv2) {
    Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
    uint8_t markerColor = 0;
    long leafCnt = 0;
    int surfelCnt = 0;
    int decimation = 10;

    //  Get all the surfels possible
    for (auto surfel : surfelsv2) {
      // Change color

      if (surfelCnt++ % decimation != 0)
        continue;

      if (++markerColor > 14)
        markerColor = 0;

      if(surfelCnt > 500 * decimation)
        break;
      // Get the surfels
      for (auto& leaf : surfel.leafs_) {
        // Decimate for Rviz
        // if (leafCnt++ % decimation != 0)
        //   continue;

        // Set the translation part
        surfelPose.translation() = Eigen::Vector3d(leaf->mean_);
        // Calculate the rotation matrix
        Eigen::Matrix3d rotMatrix = matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), leaf->eigenvectors_.col(0));
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
          for (const Eigen::Isometry3d& pose : surfel->odomPoses_) {
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

  void PointCloudProc::visualizeCorrespondingSurfelsV2WithPoses(std::vector<Surfelv2>& surfelsv2) {

    unsigned int maxLeafs = 0;
    unsigned int maxLeafsId = 0;
    for (unsigned int i = 0; i < surfelsv2.size(); i++) {
      if (surfelsv2.at(i).leafs_.size() > maxLeafs) {
        maxLeafs = surfelsv2.at(i).leafs_.size();
        maxLeafsId = i;
      }
    }

    // for (unsigned int i = 0; i < surfelsv2.size(); i += decimation) {
      static int markerColor = 9;
      for (auto const& leaf : surfelsv2.at(maxLeafsId).leafs_) {
        static int visCnt = 0;
          visualizeSurfel(leaf, markerColor);
          Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
          trans = poses_.at(leaf->pointcloud_id_);
          visual_tools_->publishLine(trans.translation(), leaf->mean_, static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::LARGE);
          visual_tools_->trigger();
      }
    // }
  }

  void PointCloudProc::visualizeCorrespondingSurfelsWithPoses() {

      int decimation = 1;
      int surfelsToVis = 10000;

      unsigned int maxPoses = 0;
      for (unsigned int i = 0; i < surfels_.size(); i++) {
          if (surfels_.at(i)->odomPoses_.size() > maxPoses) {
              maxPoses = surfels_.at(i)->odomPoses_.size();
          }
      }
      // Add all poses for given surfel
      geometry_msgs::PoseArray poseArray;
      poseArray.header.stamp = ros::Time::now();
      poseArray.header.frame_id = "map";
      for (unsigned int i = 0; i < surfels_.size(); i+= decimation) {
        //   for (const Eigen::Isometry3d& pose : surfels_.at(i)->poses_) {
        //       // Create new pose Msg
        //       geometry_msgs::Pose poseMsg;
        //       poseMsg.position.x = pose.translation().x();
        //       poseMsg.position.y = pose.translation().y();
        //       poseMsg.position.z = pose.translation().z();
        //       // Add orientation
        //       Eigen::Quaterniond quat(pose.linear());
        //       poseMsg.orientation.x = quat.x();
        //       poseMsg.orientation.y = quat.y();
        //       poseMsg.orientation.z = quat.z();
        //       poseMsg.orientation.w = quat.w();
        //       // Add pose to poseArray
        //       poseArray.poses.push_back(poseMsg);
        //   }
          // Publish poses array
        //   poseArrayPub_.publish(poseArray);
          static int markerColor = 9;
          markerColor++;
          // Show all correspondences in one color
          for (auto const& poseId : surfels_.at(i)->poseSurfelsIds_) {
              //   Visuzalize surfels with poses num > maxPoses/2
              // ToDo check if these are different poses or sth
              // if (surfels_.at(i)->odomPoses_.size() >= 6)
              {
                  static int visCnt = 0;
                  if (visCnt++ > surfelsToVis)
                      break;
                  for (auto const& surfelId : poseId.second) {
                      TreeNodeType* leaf = kdTreeLeafes_[poseId.first].at(surfelId);
                      visualizeSurfel(leaf, markerColor % 14);
                      // Add line between pose and surfel
                      // Eigen::Isometry3d trans = surfels_.at(i)->odomPoses_.at(poseId.first);
                      Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
                      if (posesInGraph_.size() == 0)
                        trans = poses_.at(poseId.first);
                      else
                        trans = posesInGraph_.at(poseId.first);

                      visual_tools_->publishLine(trans.translation(), leaf->mean_, static_cast<rviz_visual_tools::colors>(markerColor % 14), rviz_visual_tools::LARGE);
                      visual_tools_->trigger();
                  }
              }
          }
      }
  }

  void PointCloudProc::saveSurfelsTofile() {
      // Sort the surfels based on number of surfels its created from
      std::sort(surfels_.begin(), surfels_.end(), [](const std::shared_ptr<Surfel>& a, const std::shared_ptr<Surfel>& b) {
          return a->odomPoses_.size() < b->odomPoses_.size();
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

  void PointCloudProc::filterSurfels()
  {
    std::sort(surfels_.begin(), surfels_.end(), [](const std::shared_ptr<Surfel>& a, const std::shared_ptr<Surfel>& b) {
      return a->odomPoses_.size() > b->odomPoses_.size();
    });

    // Remove all surfels except 5
    int vecSize = surfels_.size();
    for (int i = vecSize - 1; i > 400; i--) {
      surfels_.pop_back();
    }
    std::cout << "Surfels size "  << surfels_.size() << std::endl;

    // for (auto surf: surfels_)
    // {
      // std::cout << "Odom poses:  "  << surf->odomPoses_.size() << std::endl;
    // }
  }

  void PointCloudProc::mergeSurfels() {
    srrg2_core::Chrono ch1("mergeSurfels", &_timings, false);
    // Parameters for merging
    // Max distance between two surfels to consider the merge
    double maxDistance = 0.3;
    double maxDistanceNorm = 0.6;
    // Max angle between two surfels to consider the merge
    double maxAngle = 5 * M_PI / 180.0;

    //   Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
      // u_long markerColor = 0;
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
                  TreeNodeType* leafJ;
                  {
                    srrg2_core::Chrono ch2("bestMatchingLeafFast: ", &_timings, false);
                    leafJ = kdTrees_.at(j)->bestMatchingLeafFast(leafI->mean_);
                  }
                //   std::cout << "Mean of the leaf:  "<< leafI->mean_ << std::endl;
                  //   std::cout << "SurfelB size:  "  << surfelB->num_points_ << std::endl;
                  //   auto& surfelB = (kdTrees_.at(j))->bestMatchingLeafFast(surfelA->mean_);

                  // Skips surfels with only 1 point, as they dont have the 2nd and 3rd eigenvector 
                  // or rewrite it from the parent or use only first col of eigen matrix
                  // if (leafI->num_points_ == 1 || leafJ->num_points_ == 1)
                    // continue;
                  double distance = (leafJ->mean_ - leafI->mean_).norm();
                  double distanceNorm = abs((leafJ->mean_ - leafI->mean_).dot(leafI->eigenvectors_.col(0)));
                  if (distance < maxDistance || distanceNorm < maxDistanceNorm) {
                      double angle = angleBetween2Vectors(leafI->eigenvectors_.col(0), leafJ->eigenvectors_.col(0));
                      if (angle < maxAngle) {
                        // Visualize the surfels for merge
                        //   visualizeSurfel(leafI, markerColor % 14);
                        //   visualizeSurfel(leafJ, markerColor % 14);
                        //   markerColor++;
                          matchedSurfels++;

                          // Need to find the leafJid in the vector anyway:
                          // ToDo optimize somehow because its very slow - give ids to surfels or sth
                          int idTmp;
                          {
                            srrg2_core::Chrono ch3("findLeafId: ", &_timings, false);
                            idTmp = findLeafId(j, leafJ);                            
                          }
                          if (idTmp == -1){
                              std::cerr << "Not found id of existing leaf" << std::endl;
                              exit(0);
                          }
                          unsigned int idLeafJ = (unsigned int) idTmp;
                          
                          // If correspondence found then check if that surfel already exists
                          std::shared_ptr<Surfel> surfel;
                          bool foundSurfelI = false, foundSurfelJ = false;
                          int cnt = -1;
                          int cntI = -1, cntJ = -1;
                          bool skip = false;
                          srrg2_core::Chrono ch2("Rest of merging loop: ", &_timings, false);
                          for (std::shared_ptr<Surfel> & tmpSurfel : surfels_) {
                            cnt++;
                            // If that surfel already exists in pose I:

                              if (tmpSurfel->checkIfsurfelIdExists(i, idLeafI)) {
                                  surfel = tmpSurfel;
                                  foundSurfelI = true;
                                  // break;
                                  cntI = cnt;
                              }
                              if (tmpSurfel->checkIfsurfelIdExists(j, idLeafJ)) {
                                  surfel = tmpSurfel;
                                  foundSurfelJ = true;
                                  cntJ = cnt;
                              }
                              // If both leafs already belong to the same surfel
                              if (foundSurfelI && foundSurfelJ) {
                                if (cntI == cntJ) {
                                  skip = true;
                                  break;
                                }
                                else
                                {
                                  skip = true;
                                  break;
                                }
                              }
                          }
                            if (skip)
                            continue;

                          // Check if given surfel has already correspondence in pose I or J
                          // At this point only one foundSurfel should be true
                          if (surfel) {
                            srrg2_core::Chrono ch5("checkIfPoseExists: ", &_timings, false);
                            if (foundSurfelI)
                              if (surfel->checkIfPoseExists(j))
                                continue;

                            if (foundSurfelJ)
                              if (surfel->checkIfPoseExists(i))
                                continue;
                          }

                          // If surfel doesnt exists then create it
                          if (!surfel) {
                            surfel = std::make_shared<Surfel>();
                          }

                          // Create observation for given surfel from pose I
                          if (!foundSurfelI) {
                            srrg2_core::Chrono ch6("addingObservation: ", &_timings, false);
                            Eigen::Matrix<double, 3, 5> observationI = Eigen::Matrix<double, 3, 5>::Identity();
                            observationI.block<3, 3>(0, 0) = leafI->eigenvectors_;
                            observationI.block<3, 1>(0, 3) = leafI->mean_;
                            observationI.block<3, 1>(0, 4) = leafI->bbox_;
                            // if (posesInGraph_.size() == 0)
                            // surfel->addObservation(poses_.at(i), i, observationI);
                            // else
                              surfel->addObservation(posesInGraph_.at(i), i, observationI);
                          }

                          // Create observation for given surfel from pose J
                          if (!foundSurfelJ) {
                            srrg2_core::Chrono ch7("addingObservation: ", &_timings, false);
                            Eigen::Matrix<double, 3, 5> observationJ = Eigen::Matrix<double, 3, 5>::Identity();
                            observationJ.block<3, 3>(0, 0) = leafJ->eigenvectors_;
                            observationJ.block<3, 1>(0, 3) = leafJ->mean_;
                            observationJ.block<3, 1>(0, 4) = leafJ->bbox_;
                            // if (posesInGraph_.size() == 0)
                              // surfel->addObservation(poses_.at(j), j, observationJ);
                            // else
                              surfel->addObservation(posesInGraph_.at(j), j, observationJ);
                          }

                          // ToDo: make Surfel method for this
                          // Add leafI in a kdTreeI
                          surfel->poseSurfelsIds_[i].insert(idLeafI);
                          // Add leafJ in a kdTreeJ
                          surfel->poseSurfelsIds_[j].insert(idLeafJ);
                          // Add surfel to a vector for storage
                          if (foundSurfelI == false && foundSurfelJ == false)
                              surfels_.push_back(std::move(surfel));
                      }
                  }
              }
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

      // Generate the TF message based on odometry msgs
      static uint cnt = 0;
      geometry_msgs::TransformStamped tfStamped;
      tfStamped.header = rosMsgPtr->header;
      //   tfStamped.child_frame_id = rosMsgPtr->child_frame_id + "_" + std::to_string(rosMsgPtr->header.stamp.toSec());
      tfStamped.child_frame_id = rosMsgPtr->child_frame_id + "_" + std::to_string(cnt++);
      tfStamped.transform.translation.x = rosMsgPtr->pose.pose.position.x;
      tfStamped.transform.translation.y = rosMsgPtr->pose.pose.position.y;
      tfStamped.transform.translation.z = rosMsgPtr->pose.pose.position.z;
      tfStamped.transform.rotation = rosMsgPtr->pose.pose.orientation;
      // transformBroadcaster_.sendTransform(tfStamped);
  }

  void PointCloudProc::handleCloudMessage(PointCloud2MessagePtr cloudMsg) {

      // Create buffer for messages
      static std::vector<PointCloud2MessagePtr> cloudBuffer;
      // Add item to a buffer
      cloudBuffer.push_back(cloudMsg); 
      // Try to empty the buffer
      // Do not increment the i, as the cloudBuffer size decreases if the point cloud is processed
      for (unsigned int i = 0; i < cloudBuffer.size();) { 
          // Check if there is already pose for that cloud
          // Get the index that this cloud will have
          int idx_cloud = pointClouds_.size();
          int idx_pose = poses_.size() - 1;

          // If there is no pose for the cloud then add it to a buffer (pointClouds_ is being pushed back at the end of this func)
          if (idx_cloud > idx_pose) {
            //   cloudBuffer.push_back(std::shared_ptr(cloud));
              std::cout << "No pose for the pointcloud   |  Buffer size:  " << cloudBuffer.size() << std::endl;
              return;
          }

          // Get the reference to first item in buffer
          PointCloud2MessagePtr& cloud = cloudBuffer.at(i);

          // Count the number of point clouds for intensity value and the frame_id
          static uint cloudNum = 0;

          // Change the frame_id of the point cloud
          cloud->frame_id.value() = "ba_estimate_" + std::to_string(cloudNum++);
          //   cloud->frame_id.value() = "ba_estimate_" + std::to_string(cloud->timestamp.value());;

          // Create the point cloud to set new intensity value
          std::shared_ptr<PointIntensity3fVectorCloud> pointCloudI3f(new PointIntensity3fVectorCloud());

          // Get the point from original point cloud
          cloud->getPointCloud(*pointCloudI3f);

          // Set the Intensity for Rviz visualization
          for (PointIntensity3f& p : *pointCloudI3f) {
            p.intensity() = (float) cloudNum;
          }
          // Set point to the visualizaton point cloud - overrides the intensity of the original cloud
          cloud->setPointCloud(*pointCloudI3f);

          // Convert srrg2 to sensor_msgs point cloud
          sensor_msgs::PointCloud2ConstPtr rosMsg = Converter::convert(cloud);

          // Store the point cloud to visualize it after optimization etc.
          rosPointClouds_.push_back(*rosMsg);

          // Publish the raw cloud as ROS message
        //   pointCloudPub_.publish(*rosMsg);

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
          createKDTree(pointCloudEigen, poses_.at(idx_cloud));
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

      uint8_t decimation = 1;

      // Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      for (auto& leaf : kdTreeLeafes_.at(idx)) {
        // Decimate for Rviz
        if (static int cnt = 0; cnt++ % decimation != 0)
          continue;
        if (++markerColor > 14)
          markerColor = 0;

        visualizeSurfel(leaf, markerColor);
      }
  }

  void PointCloudProc::reloadRviz() {
    rviz::SendFilePath srv;
    srv.request.path.data = ros::package::getPath("structure_refinement") + "/rviz_config/structure_refinement.rviz";
    if (clientRviz_.call(srv)) {
      ROS_INFO("Sum: %d", (bool)srv.response.success);
    } else {
      ROS_ERROR("Failed to call service reloadRviz");
    }
  }

  void PointCloudProc::createKDTree(std::vector<Eigen::Vector3d>& cloud, const Eigen::Isometry3d& lastPose) {
      using ContainerType = std::vector<Eigen::Vector3d>;
      using TreeNodeType = TreeNode3D<ContainerType>;
      using TreeNodeTypePtr = TreeNodeType*;

      static int pointCloudId = -1;
      pointCloudId++;
      // Create kdTree from the point cloud
      std::shared_ptr<TreeNodeType> kdTree(new TreeNodeType(cloud.begin(),
                                              cloud.end(),
                                              0.2,
                                              0.1,
                                              0,
                                              2,
                                              nullptr,
                                              nullptr,
                                              pointCloudId));

      // Transform kdTree according to the BA pose
      // Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
      // Eigen::Vector3d trans = Eigen::Vector3d(20,20,20);
      // The surfels should be transformed with the noise, so that they are consistent in a global frame (point cloud is in local)
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

  void PointCloudProc::generateSyntheticOdometry(nav_msgs::Odometry & odomMsg) {

      Eigen::Isometry3d poseInc = Eigen::Isometry3d::Identity();
      poseInc.translation() = Eigen::Vector3d(1.0, 1.0, 0.01);
      double roll = 0.01 * 3, pitch = 0.03 * 2, yaw = 0.05;
      poseInc.linear() = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                             .toRotationMatrix();

      // Change inc direction after 5 poses
      static int cnt = 0;
      if (cnt++ > 5)
        poseInc.translation().x() *= -1;

      // Calculate new pose
      Eigen::Isometry3d newPose = Eigen::Isometry3d::Identity();
      if (poses_.size() != 0)
          newPose = poses_.back() * poseInc;
      else{
          // Initial pose
          newPose.translation() = Eigen::Vector3d(3, 2, 2);
          newPose.linear() = poseInc.linear();
      }

      // Fill up the odometry message
      Eigen::Quaterniond quat(newPose.linear());

      // Normalize the quaternion
      quat.normalize();
      
      odomMsg.header.frame_id = "map";
      odomMsg.header.stamp = ros::Time::now();
      lastTimestamp_ = odomMsg.header.stamp;
      odomMsg.child_frame_id = "ba_estimate";
      odomMsg.pose.pose.position.x = newPose.translation().x();
      odomMsg.pose.pose.position.y = newPose.translation().y();
      odomMsg.pose.pose.position.z = newPose.translation().z();
      odomMsg.pose.pose.orientation.x = quat.x();
      odomMsg.pose.pose.orientation.y = quat.y();
      odomMsg.pose.pose.orientation.z = quat.z();
      odomMsg.pose.pose.orientation.w = quat.w();
  }

  void PointCloudProc::generateSyntheticPointCloud(sensor_msgs::PointCloud2 & cloudMsg) {
      
      static std::random_device rd;                           // obtain a random number from hardware
      static std::mt19937 gen(rd());                          // seed the generator
      static std::normal_distribution<double> noise(0, 0.03);  // noise (mean, var)

      // Synthethic cloud
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pointCloudSynth;

      // Room dimensons
      double width = 10, depth = 15, height = 5;
      double pointsDensity = 5;  // Pts per meter
      Eigen::Vector3d point;
      // Generate floor
      for (int i = 0; i <= width * pointsDensity; i++) {
          for (int j = 0; j <= depth * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3d(i / pointsDensity + noise(gen), j / pointsDensity + noise(gen), 0 + noise(gen)));
          }
      }
      // Generate front and back walls
      for (int i = 0; i <= width * pointsDensity; i++) {
          for (int j = 1; j < height * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3d(i / pointsDensity + noise(gen), 0 + noise(gen), j / pointsDensity + noise(gen)));
              pointCloudSynth.push_back(Eigen::Vector3d(i / pointsDensity + noise(gen), depth + noise(gen), j / pointsDensity + noise(gen)));
          }
      }

      // Generate left and right walls
      for (int i = 1; i < depth * pointsDensity; i++) {
          for (int j = 1; j < height * pointsDensity; j++) {
            pointCloudSynth.push_back(Eigen::Vector3d(0 + noise(gen), i / pointsDensity + noise(gen), j / pointsDensity + noise(gen)));
            pointCloudSynth.push_back(Eigen::Vector3d(width + noise(gen), i / pointsDensity + noise(gen), j / pointsDensity + noise(gen)));
          }
      }

      // Fill up the message fields
      cloudMsg.header.frame_id = "ba_estimate";
      //   cloudMsg.header.stamp = ros::Time::now();
      cloudMsg.header.stamp = lastTimestamp_;
      cloudMsg.height = 1;
      cloudMsg.width = pointCloudSynth.size();
      cloudMsg.is_bigendian = false;
      cloudMsg.is_dense = true;

      // Get the last pose to transform the pose to local LiDar frame
      Eigen::Isometry3d & lastSynthPose = poses_.back();
      // Create the modifier to setup the fields and memory
      sensor_msgs::PointCloud2Modifier cloudModifier(cloudMsg);
      // Set the fields that our cloud will have
      //   cloudModifier.setPointCloud2FieldsByString(1, "xyz");
      cloudModifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                         "y", 1, sensor_msgs::PointField::FLOAT32,
                                         "z", 1, sensor_msgs::PointField::FLOAT32,
                                         "intensity", 1, sensor_msgs::PointField::FLOAT32);
      // Create iterators
      sensor_msgs::PointCloud2Iterator<float> iterX(cloudMsg, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(cloudMsg, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(cloudMsg, "z");
      sensor_msgs::PointCloud2Iterator<float> iterI(cloudMsg, "intensity");

      for (auto& point : pointCloudSynth) {
          if (iterX != iterX.end()) {
              Eigen::Vector3f point3f =  (lastSynthPose.inverse() * point).cast<float>();
              *iterX = point3f[0];
              *iterY = point3f[1];
              *iterZ = point3f[2];
              ++iterX;
              ++iterY;
              ++iterZ;
          } else
              break;
      }
    //   synthPointCloudPub_.publish(cloudMsg);
    //   visual_tools_->publishSpheres(pointCloudSynth, static_cast<rviz_visual_tools::colors>(9), rviz_visual_tools::SMALL);
    //   visual_tools_->trigger();
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

  void PointCloudProc::createFactorGraph(srrg2_solver::FactorGraphPtr &graph) {

    // Create graph

    // Add first variable
    auto firstPoseVar = std::make_shared<srrg2_solver::VariableSE3QuaternionRight>();
    firstPoseVar->setGraphId(0);
    firstPoseVar->setEstimate(poses_.at(0).cast<float>());
    firstPoseVar->setStatus(srrg2_solver::VariableBase::Status::Fixed);
    graph->addVariable(firstPoseVar);
    // Save the previous variable for next iterations
    auto prevPoseVar = firstPoseVar;

    // Define the noise added to the poses
    Eigen::Isometry3f perturbation = Eigen::Isometry3f::Identity();
    static std::random_device rd;                                 // obtain a random number from hardware
    static std::mt19937 gen(rd());                                // seed the generator
    static std::normal_distribution<double> noise(0.1, 0.2);  //  // define the range

    // Take all poses
    for (uint idx = 1; idx < poses_.size(); idx++) {
      // Create a new variable for each pose
      auto poseVar = std::make_shared<srrg2_solver::VariableSE3QuaternionRight>();
      // Set Id of a varaible
      poseVar->setGraphId(idx);

      // Generate the noise matrix
      perturbation.translation() = Eigen::Vector3f(noise(gen), noise(gen), noise(gen));
      perturbation.linear() = (Eigen::AngleAxisf(noise(gen), Eigen::Vector3f::UnitX()) *
                               Eigen::AngleAxisf(noise(gen), Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(noise(gen), Eigen::Vector3f::UnitZ()))
                                  .toRotationMatrix();

      // Set estimate with noise
      // poseVar->setEstimate(poses_.at(idx).cast<float>() * perturbation);
      poseVar->setEstimate(poses_.at(idx).cast<float>());

      // Add variable to the graph
      graph->addVariable(srrg2_solver::VariableBasePtr(poseVar));

      // Create factorype();
      auto factor = std::make_shared<srrg2_solver::SE3PosePoseGeodesicErrorFactor>();
      factor->setVariableId(0, prevPoseVar->graphId());
      factor->setVariableId(1, poseVar->graphId());

      // Set the measurement based on GT data, without the perturbation
      Eigen::Isometry3f inc = poses_.at(idx-1).cast<float>().inverse() * poses_.at(idx).cast<float>();
      factor->setMeasurement(inc);

      // Set information matrix as Identity
      Eigen::Matrix<float, 6, 6> infMat = Eigen::Matrix<float, 6, 6>::Identity();
      factor->setInformationMatrix(infMat);

      // Add factor to the graph
      graph->addFactor(factor);

      prevPoseVar = poseVar;
    }
    
    // Simulates loop closure for test
    // {
    //   auto factor = std::make_shared<srrg2_solver::SE3PosePoseGeodesicErrorFactor>();
    //   factor->setVariableId(0, firstPoseVar->graphId());
    //   factor->setVariableId(1, poses_.size() - 1);
    //   factor->setMeasurement(Eigen::Isometry3f::Identity());
    //   Eigen::Matrix<float, 6, 6> infMat = Eigen::Matrix<float, 6, 6>::Identity();
    //   factor->setInformationMatrix(infMat);
    //   graph->addFactor(factor);
    // }

  }

  // BA-like factor graph
  void PointCloudProc::handleFactorGraphBA() {
    srrg2_core::Chrono ch1("handleFactorGraphBA: ", &_timings, false);
    // Create graph variable
    srrg2_solver::FactorGraphPtr graph(new srrg2_solver::FactorGraph);
    std::vector<Eigen::Isometry3d> surfelGTVector;
    std::vector<SynthSurfel> surfelsAfterRawOptim;
    for (int i = 0; i < 1; i++) {
      std::cout << "Iteration:  "  << i << std::endl;
      // Clear the graph
      graph->clear();
      // Clear existing surfels (surfel correspondences)
      surfels_.clear();

      // Add poses
      if (i == 0)
        addPosesToGraphBA(graph, poses_);
      else
        addPosesToGraphBA(graph, posesInGraph_);

      publishTFFromGraph(graph);
      publishPointClouds();
      publishTFFromGraph(graph);

      // Save actual values of poses to calculate the increments afterwards
      // Update it here, so that in mergeSurfels() function I can use posesInGraph_
      updatePosesInGraph(graph);
      // Merge and visualize surfels
      mergeSurfels();

      // visualizeCorrespondingSurfelsWithPoses();
      // ros::Duration(1.0).sleep();

      // Add surfels
      addSurfelsToGraphBA(graph);
      visual_tools_->deleteAllMarkers();
      publishSurfFromGraph(graph);

      // Just for test - synthethic Surfel
      // Generate multiple surfels
      // visual_tools_->deleteAllMarkers();
      // std::vector<SynthSurfel> surfelsWithNoise;
      // surfelGTVector = addSynthSurfelsToGraphBA(graph, surfelsWithNoise);
      // publishSurfFromGraph(graph);

      // updatePosesInGraph(graph);

      // Visualize point clouds
          //  ros::Duration(3.0).sleep();
      // publishTFFromGraph(graph);
      // publishPointClouds();
      // publishTFFromGraph(graph);


      // Optimize the graph
      optimizeFactorGraph(graph);
      rawOptimizeSurfels(surfels_);

      // Update kdTree and leafs (surfels) in the map frame, based on actual poses and poses before optimization
      // Only necessary when there are more than one iteration of merging the surfels
      // updateLeafsPosition(graph, posesInGraph_);

      // Update for next iteration
      updatePosesInGraph(graph);
      ros::Duration(5.0).sleep();
    }
    srrg2_core::Chrono ch2("Visualizations after optim.: ", &_timings, false);

    // Visualize point clouds
    publishTFFromGraph(graph);
    publishPointClouds();
    publishTFFromGraph(graph);

    // Visualize surfels
    visual_tools_->deleteAllMarkers();
    publishSurfFromGraph(graph);

    // Calculate the error between poses after optimization and GT
    int poseCnt = 0;
    for (auto& pose : posesInGraph_) {
      // Use poses_ here instead of posesWithoutNoise_, so that the GT poses are when the point clouds are aligned (for visialization only)
      Eigen::Isometry3d error = posesWithoutNoise_.at(poseCnt).inverse() * pose;
      std::cout << std::fixed << "Error for pose " << std::setprecision(2) <<  poseCnt << std::endl
                << error.matrix() << std::endl;
      poseCnt++;
    }

    // // Calculate the error between surfels after optimization and GT
    // int surfelCnt = 0;
    for (auto varIt : graph->variables()) {
      srrg2_solver::VariableSurfelAD1D* varSurf = dynamic_cast<srrg2_solver::VariableSurfelAD1D*>(varIt.second);
      if (!varSurf) {
        continue;
      }
      static int cnt = 0;
      std::cout << std::setprecision(5) <<  "Diff between surfels optim " 
      << (varSurf->estimate().translation() - surfels_.at(cnt)->estPosition_.cast<float>()).transpose() << std::endl;
      if (cnt++ > 10)
        break;
    }
    //   Eigen::Isometry3f error = surfelGTVector.at(surfelCnt).inverse().cast<float>() * varSurf->estimate();
      
    //   Eigen::Isometry3f errorRaw = surfelGTVector.at(surfelCnt).inverse().cast<float>() * surfelsAfterRawOptim.at(surfelCnt).getIsometry();
    //   std::cout << std::fixed << "Error for surfel " << std::setprecision(2) << surfelCnt << std::endl
    //             << error.matrix() << std::endl;
    //   std::cout << std::fixed << "Error for surfel optimization without solver " << std::setprecision(2) << surfelCnt << std::endl
    //             << errorRaw.matrix() << std::endl;

    //    std::cout << std::fixed << "Matrix GT " << std::setprecision(2) << std::endl
    //             << surfelGTVector.at(surfelCnt).matrix() << std::endl;

    //    std::cout << std::fixed << "Matrix Solver " << std::setprecision(2)  << std::endl
    //             << varSurf->estimate().matrix() << std::endl;

    //    std::cout << std::fixed << "Matrix No Solver " << std::setprecision(2)  << std::endl
    //              << surfelsAfterRawOptim.at(surfelCnt).getIsometry().matrix() << std::endl << std::endl;

    //    surfelCnt++;
    // }
  }

  void PointCloudProc::handleFactorGraphAfterGPU(std::vector<Surfelv2>& surfelsv2) {
    // Create graph variable
    srrg2_solver::FactorGraphPtr graph(new srrg2_solver::FactorGraph);
    {
      srrg2_core::Chrono chGP2("Adding poses to graph", &_timings, false);
      addPosesToGraphBA(graph, poses_);
    }

    publishTFFromGraph(graph);
    publishPointClouds();
    publishTFFromGraph(graph);

    // Add surfels
    {
      srrg2_core::Chrono chGP2("Adding surfels to graph", &_timings, false);
      addSurfelsToGraphAfterGPU(graph, surfelsv2);
    }
    visual_tools_->deleteAllMarkers();

    publishSurfFromGraph(graph);
    publishPointSurfv2(surfelsv2);

    // Optimize the graph
    {
      srrg2_core::Chrono chGP2("Optimizing the graph", &_timings, false);
      optimizeFactorGraph(graph);
    }


    updateSurfelsMeanFromGraph(graph, surfelsv2);
    // Update poses and kd-tree leafs
    updatePosesAndLeafsFromGraph(graph);

    // rawOptimizeSurfels(surfels_);

    // Update for next iteration
    // updatePosesInGraph(graph);
    // ros::Duration(10.0).sleep();

    // Visualize point clouds
    publishTFFromGraph(graph);
    publishPointClouds();
    publishTFFromGraph(graph);

    publishPointSurfv2(surfelsv2);

    // Visualize surfels
    visual_tools_->deleteAllMarkers();
    // publishSurfFromGraph(graph);
    
  }

  void PointCloudProc::updatePosesAndLeafsFromGraph(srrg2_solver::FactorGraphPtr& graph) {
    posesInGraph_.clear();
    for (uint i = 0; i < poses_.size(); i++) {
      srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(i));
      if (!varPose) {
        std::cout << "Error in updatePosesInGraph" << std::endl;
        exit(0);
      }
      Eigen::Isometry3d poseInc = varPose->estimate().cast<double>() * poses_.at(i).inverse();
      if(i < kdTrees_.size())
        kdTrees_.at(i)->applyTransform(poseInc.linear(), poseInc.translation());
      poses_.at(i) = varPose->estimate().cast<double>();
    }
  }

void PointCloudProc::updatePosesInGraph(srrg2_solver::FactorGraphPtr& graph) {
  posesInGraph_.clear();
  for (uint i = 0; i < poses_.size(); i++) {
    srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(i));
    if (!varPose) {
      std::cout << "Error in updatePosesInGraph" << std::endl;
      exit(0);
    }
    posesInGraph_.push_back(varPose->estimate().cast<double>());

  }
}

std::vector<Eigen::Isometry3d> PointCloudProc::addSynthSurfelsToGraphBA(srrg2_solver::FactorGraphPtr& graph, std::vector<SynthSurfel> & surfOut) {

  int64_t lastGraphId = poses_.size() - 1;

  std::vector<Eigen::Isometry3d> surfelGTVector;

  // Used to generate random position of a surfel
  Eigen::Isometry3d surfelGT = Eigen::Isometry3d::Identity();
  static std::random_device rdS;                            // obtain a random number from hardware
  static std::mt19937 genS(rdS());                          // seed the generator
  static std::normal_distribution<double> trans(5.0, 8.0);  // define the noise for translation
  static std::normal_distribution<double> rot(0.0, M_PI);    // define the noise for rotation

  // Generate the surfels GT positions vector
  for (int j = 0; j < 20; j++) {

    // Random position of a surfel
    surfelGT.translation() = Eigen::Vector3d(trans(genS), trans(genS), trans(genS));
    surfelGT.linear() = (Eigen::AngleAxisd(rot(genS), Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(rot(genS), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rot(genS), Eigen::Vector3d::UnitZ()))
                            .toRotationMatrix();
    surfelGTVector.push_back(surfelGT);
  }

  for (int j = 0; j < surfelGTVector.size(); j++) {
    auto surfelVar = std::make_shared<srrg2_solver::VariableSurfelAD1D>();
    Eigen::Isometry3d surfelPoseGT = surfelGTVector.at(j);
    surfelVar->setEstimate(surfelPoseGT.cast<float>());
    surfelVar->setGraphId(++lastGraphId);
    // surfelVar->setStatus(srrg2_solver::VariableBase::Status::Fixed);
    graph->addVariable(surfelVar);

    // Create synthethic
    SynthSurfel synthSurfel;

      static int markerColor = 0;
      markerColor++;
    for (int i = 0; i < poses_.size(); i++) {
      srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(i));

      // Create factor
      std::shared_ptr<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D> poseSurfelFactor = std::make_shared<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D>();
      poseSurfelFactor->setVariableId(0, i);
      poseSurfelFactor->setVariableId(1, surfelVar->graphId());

      Eigen::Isometry3f surfelInPose = varPose->estimate().inverse() * surfelPoseGT.cast<float>();

      // Add noise
      static std::random_device rd;                                              // obtain a random number from hardware
      static std::mt19937 gen(rd());                                             // seed the generator
      static std::normal_distribution<float> noiseTrans(0.0, 0.2);               // define the noise for translation
      static std::normal_distribution<float> noiseRot(0.0, 0.0 * M_PI / 180.0);  // define the noise for rotation
      Eigen::Isometry3f perturbation = Eigen::Isometry3f::Identity();
      perturbation.translation() = Eigen::Vector3f(0,0, noiseTrans(gen));
      perturbation.linear() = (Eigen::AngleAxisf(noiseRot(gen), Eigen::Vector3f::UnitX()) *
                               Eigen::AngleAxisf(noiseRot(gen), Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(noiseRot(gen), Eigen::Vector3f::UnitZ()))
                                  .toRotationMatrix();

      // Add noise to the measurement
      surfelInPose = surfelInPose * perturbation;
      poseSurfelFactor->setMeasurement(surfelInPose);
      Eigen::Isometry3f surfelInMap = varPose->estimate() * surfelInPose;
      visual_tools_->publishAxis(surfelInMap.cast<double>(), rviz_visual_tools::LARGE);
      visual_tools_->publishLine(varPose->estimate().translation().cast<double>(), surfelInMap.translation().cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor%14));
      visual_tools_->trigger();

      // Add observation to synth surfel
       Eigen::Matrix<double,3,2> obs;
        // Position of a surfel
      obs.block<3,1>(0,0) = surfelInMap.translation().cast<double>();
        // Normal of a surfel
      obs.block<3,1>(0,1) = surfelInMap.linear().col(2).cast<double>();
      synthSurfel.addObservation(obs);

      // Set information matrix
      Eigen::Matrix<float, 1, 1> infMat = Eigen::Matrix<float, 1, 1>::Identity();
      poseSurfelFactor->setInformationMatrix(infMat);

      // Add factor to the graph
      graph->addFactor(poseSurfelFactor);

      // In the last iteration add noise to the inital estimate of surfel
      if (i == poses_.size() - 1) {
        perturbation.translation() *= 2;
        surfelVar->setEstimate(surfelPoseGT.cast<float>() * perturbation);
        Eigen::Matrix<double,3,2> obs;
        synthSurfel.estPosition_ = surfelVar->estimate().translation().cast<double>();
        synthSurfel.estNormal_ = surfelVar->estimate().linear().col(2).cast<double>();
        surfOut.push_back(synthSurfel);
      }
      // In the last iteration add noise to the pose
      if (j == surfelGTVector.size() - 1) {
        // if (i != 0) // except the first pose
          // varPose->setEstimate(varPose->estimate() * perturbation);
      }
    }
  }
  return surfelGTVector;
}

void PointCloudProc::addSurfelsToGraphBA(srrg2_solver::FactorGraphPtr& graph){

    // Set lastID to the number of poses
    int64_t lastGraphId = poses_.size() - 1;

    // Iterate through all surfels
    for (const std::shared_ptr<Surfel>& surfel : surfels_) {

      // Set containing ids of odom poses
      std::set<uint> odomPoseIdSet;

      // Create multiple subsurfel variables for each odomPoses_
      auto surfelVar = std::make_shared<srrg2_solver::VariableSurfelAD1D>();
      surfelVar->setGraphId(++lastGraphId);

      // Set initial estimate
      // ToDo: Average all the observed surfels
      Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
      Eigen::Isometry3f rotationY = Eigen::Isometry3f::Identity();
      rotationY.linear() << 0, 0, 1, 0, 1, 0, -1, 0, 0;
      surfelPose.translation() = surfel->observations_.at(0).block<3, 1>(0, 3);
      surfelPose.linear() = matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), surfel->observations_.at(0).block<3, 1>(0, 0));
      surfelVar->setEstimate(surfelPose.cast<float>() * rotationY);

      // std::cout << "SurfelPose before optim: " << std::endl << surfelVar->estimate().matrix() << std::endl;
      // surfelVar->setStatus(srrg2_solver::VariableBase::Status::Fixed);

      graph->addVariable(surfelVar);

      // Iterate through all poses from which given surfel was registered
      for (uint i = 0; i < surfel->odomPoses_.size(); i++) {

        // Detect if given surfel has 2 or more observations from the same pose
        uint odomPoseId = surfel->odomPosesIds_.at(i);
        if (odomPoseIdSet.find(odomPoseId) != odomPoseIdSet.end())
          continue;
        // Add that pose id to a set
        else
          odomPoseIdSet.insert(odomPoseId);

        // Create factor between pose and the surfel
        // Eigen::Isometry3d surfelOdomPose = surfel->odomPoses_.at(i);
        std::shared_ptr<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D> poseSurfelFactor = std::make_shared<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D>();
        poseSurfelFactor->setVariableId(0, (int64_t)odomPoseId);
        poseSurfelFactor->setVariableId(1, surfelVar->graphId());
        // poses_ .at(odomPoseId) should be the same as surfel->odomPoses_(i)
        if (posesInGraph_.at(odomPoseId).matrix() != surfel->odomPoses_.at(i).matrix())
          std::cout << "Error: Poses stored in surfel and saved as odometetry don't match" << std::endl;

        Eigen::Isometry3f odomPose = surfel->odomPoses_.at(i).cast<float>();
        Eigen::Isometry3f surfelInMap = Eigen::Isometry3f::Identity();
        surfelInMap.translation() = surfel->observations_.at(i).block<3,1>(0,3).cast<float>();
        surfelInMap.linear() = matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), surfel->observations_.at(i).block<3, 1>(0, 0)).cast<float>();

        poseSurfelFactor->setMeasurement(odomPose.inverse() * surfelInMap * rotationY);

        // Set information matrix
        Eigen::Matrix<float, 1, 1> infMat = Eigen::Matrix<float, 1, 1>::Identity();
        infMat *= 1;
        poseSurfelFactor->setInformationMatrix(infMat);
        // poseSurfelFactor->setInformationMatrix()

        // Add factor to the graph
        graph->addFactor(poseSurfelFactor);
        // std::cout << "Added factor to graph" << std::endl;
        // Connect subsurfels themselfes
        // auto surfelSurfelFactor = std::make_shared<srrg2_solver::SE3Plane2PlaneErrorFactor>();
        // surfelSurfelFactor->setVariableId(0, surfelVar->graphId()-1);
        // surfelSurfelFactor->setVariableId(1, surfelVar->graphId());

        // Eigen::Isometry3f surfelInSurfel = (surfel->odomPoses_.at(i).inverse() * surfelPose).cast<float>();
        // surfelSurfelFactor->setMeasurement(surfelInSurfel);

        // Set information matrix as Identity
        // Eigen::Matrix<float, 4, 4> infMat = Eigen::Matrix<float, 4, 4>::Identity();
        // surfelSurfelFactor->setInformationMatrix(infMat);

        // Add factor to the graph
        // graph->addFactor(surfelSurfelFactor);
      }
    }
}

void PointCloudProc::addSurfelsToGraphAfterGPU(srrg2_solver::FactorGraphPtr& graph, std::vector<Surfelv2>& surfelsv2){

    // Set lastID to the number of poses
    int64_t lastGraphId = poses_.size() - 1;

    // Iterate through all surfels
    for (const Surfelv2 & surfel : surfelsv2) {

      // Set containing ids of odom poses
      // Create multiple subsurfel variables for each odomPoses_
      auto surfelVar = std::make_shared<srrg2_solver::VariableSurfelAD1D>();
      surfelVar->setGraphId(++lastGraphId);

      // Set initial estimate
      // ToDo: Average all the observed surfels
      Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
      Eigen::Isometry3f rotationY = Eigen::Isometry3f::Identity();
      rotationY.linear() << 0, 0, 1, 0, 1, 0, -1, 0, 0;
      surfelPose.translation() = surfel.leafs_.at(0)->mean_;
      surfelPose.linear() = matrixBetween2Vectors(Eigen::Vector3d(0, 0, 1), surfel.leafs_.at(0)->eigenvectors_.col(0));
      surfelVar->setEstimate(surfelPose.cast<float>());

      // std::cout << "SurfelPose before optim: " << std::endl << surfelVar->estimate().matrix() << std::endl;
      // surfelVar->setStatus(srrg2_solver::VariableBase::Status::Fixed);

      graph->addVariable(surfelVar);

      // Iterate through all poses from which given surfel was registered
      for (uint i = 0; i < surfel.leafs_.size(); i++) {

        // Create factor between pose and the surfel
        // Eigen::Isometry3d surfelOdomPose = surfel->odomPoses_.at(i);
        std::shared_ptr<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D> poseSurfelFactor = std::make_shared<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D>();
        poseSurfelFactor->setVariableId(0, surfel.leafs_.at(i)->pointcloud_id_);
        poseSurfelFactor->setVariableId(1, surfelVar->graphId());
        // poses_ .at(odomPoseId) should be the same as surfel->odomPoses_(i)
        // if (posesInGraph_.at(odomPoseId).matrix() != surfel->odomPoses_.at(i).matrix())
          // std::cout << "Error: Poses stored in surfel and saved as odometetry don't match" << std::endl;

        Eigen::Isometry3f odomPose = poses_.at(surfel.leafs_.at(i)->pointcloud_id_).cast<float>();
        Eigen::Isometry3f surfelInMap = Eigen::Isometry3f::Identity();
        surfelInMap.translation() = surfel.leafs_.at(i)->mean_.cast<float>();
        surfelInMap.linear() = matrixBetween2Vectors(Eigen::Vector3d(0, 0, 1), surfel.leafs_.at(0)->eigenvectors_.col(0)).cast<float>();

        poseSurfelFactor->setMeasurement(odomPose.inverse() * surfelInMap);

        // Set information matrix
        Eigen::Matrix<float, 1, 1> infMat = Eigen::Matrix<float, 1, 1>::Identity();
        infMat *= 1;
        poseSurfelFactor->setInformationMatrix(infMat);
        // poseSurfelFactor->setInformationMatrix()

        // Add factor to the graph
        graph->addFactor(poseSurfelFactor);
      }
    }
}

void PointCloudProc::addPosesToGraphBA(srrg2_solver::FactorGraphPtr& graph, std::vector<Eigen::Isometry3d>& poseVect) {

  // Take all poses
  for (uint idx = 0; idx < poseVect.size(); idx++) {
    // Create a new variable for each pose
    auto poseVar = std::make_shared<srrg2_solver::VariableSE3QuaternionRightAD>();
    // Set Id of a varaible
    poseVar->setGraphId(idx);

    // Set first variable as Fixed
    if (idx == 0)
      poseVar->setStatus(srrg2_solver::VariableBase::Status::Fixed);

    // Set estimate with noise
    poseVar->setEstimate(poseVect.at(idx).cast<float>());

    // Add variable to the graph
    graph->addVariable(poseVar);

  }
}

void PointCloudProc::handleFactorGraph()
{
    // Create graph variable
    srrg2_solver::FactorGraphPtr graph(new srrg2_solver::FactorGraph);
    std::cout << "2" << std::endl;

    // Set all pose variables and pose-pose factors | Also adds noise to the estimates
    createFactorGraph(graph);
    std::cout << "2a" << std::endl;
    // Set all factors related to the surfels
    addSurfelFactors(graph);
    std::cout << "2b" << std::endl;

    // Visualize poses before the ptimization
    geometry_msgs::PoseArray poseArrayBefore;
    createPoseArrayfromGraph(poseArrayBefore, graph);
    beforeOptimPoseArrayPub_.publish(poseArrayBefore);
    // solver.saveGraph("before.graph");
    std::cout << "2c" << std::endl;

    // Visualize the point cloud before the optimizaton but after adding noise | To display PointCloud, the TF time must be later than time of PointCloud
    publishTFFromGraph(graph);
    publishPointClouds();
    publishTFFromGraph(graph);

    // Optimize the graph
    optimizeFactorGraph(graph);
    ros::Duration(5.0).sleep();
    std::cout << "2d" << std::endl;

    // Publish poses array after optimization
    geometry_msgs::PoseArray poseArrayAfter;
    createPoseArrayfromGraph(poseArrayAfter, graph);
    afterOptimPoseArrayPub_.publish(poseArrayAfter);

    // Update the point cloud after the optimizaton | To display PointCloud, the TF time must be later than time of PointCloud
    // publishTFFromGraph(graph);
    // publishPointClouds();
    // publishTFFromGraph(graph);
    
}

void PointCloudProc::updateLeafsPosition(srrg2_solver::FactorGraphPtr& graph, std::vector<Eigen::Isometry3d> & lastPosesInGraph) {
  for (uint i = 0; i < kdTrees_.size(); i++) {
    srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(i));
    if (!varPose) {
      continue;
    }
    // std::cout << "Id of variable   "  << varPose->graphId() << std::endl;
    Eigen::Isometry3d poseInc = varPose->estimate().cast<double>() * poses_.at(i).inverse() ;
    // poseInc.matrix() *= 0.1;
    // std::cout << "Change in pose #" << i << std::endl << poseInc.matrix() << std::endl;
    // It's enough to transform only kdTree_, as kdTreeLeafes_ contains pointers to them
    // varPose->estimate() * lastPoseInGraph.inverse() * 
    kdTrees_.at(i)->applyTransform(poseInc.linear(), poseInc.translation());  
  }
}

void PointCloudProc::rawOptimizeSynthSurfels(const std::vector<SynthSurfel> & surfelsIn, std::vector<SynthSurfel> & surfelsOut){

  // std::vector<Eigen::Isometry3d> surfelsOut;
  // surfelsOut = surfelsIn;
  srrg2_core::Chrono ch1("rawOptimizeSynthSurfels: ", &_timings, false);

  // Iterate through all surfels
  for (auto s : surfelsIn) {
    double A = 0;
    double b = 0;
    for (int i = 0; i < s.observations_.size(); i++) {
      Eigen::Vector3d measPos = s.observations_.at(i).block<3, 1>(0, 0);
      Eigen::Vector3d measNorm = s.observations_.at(i).block<3, 1>(0, 1);

      double e = s.estNormal_.dot(s.estPosition_ - measPos);
      A += 1;
      b += 1 * e;     
    }
    double dx = -b / A;
    SynthSurfel newSurf;
    newSurf = s;
    newSurf.estPosition_ += dx * s.estNormal_;
    surfelsOut.push_back(newSurf);
  }

}

void PointCloudProc::resetLeafsSurfelId(){
  for (auto & leafs: kdTreeLeafes_){
    for (auto & l: leafs){
      l->resetSurfelId();
    }
  }
}

void PointCloudProc::rawOptimizeSurfels(std::vector<std::shared_ptr<Surfel>> & surfelsIn){ //}, std::vector<SynthSurfel> & surfelsOut){

  // std::vector<Eigen::Isometry3d> surfelsOut;
  // surfelsOut = surfelsIn;
  srrg2_core::Chrono ch1("rawOptimizeSurfels: ", &_timings, false);

  // Iterate through all surfels
  for (auto s : surfelsIn) {
    double A = 0;
    double b = 0;
    // Set the initial estimates to the first observations - the same as in the case of using solver
    s->estPosition_ = s->observations_.at(0).block<3, 1>(0, 3);
     s->estNormal_ = s->observations_.at(0).col(0); // block<3, 1>(0, 1);
    // s.estNormal_ = s.observations_.at(0).block<3, 1>(0, 1);

    for (int i = 0; i < s->observations_.size(); i++) {
      Eigen::Vector3d measPos = s->observations_.at(i).block<3, 1>(0, 3);
      Eigen::Vector3d measNorm = s->observations_.at(i).block<3, 1>(0, 0);

      double e = s->estNormal_.dot(s->estPosition_ - measPos);
      A += 1;
      b += 1 * e;     
    }
    double dx = -b / A;
    s->estPosition_ += dx * s->estNormal_;
  }

}

 void PointCloudProc::optimizeFactorGraph(srrg2_solver::FactorGraphPtr &graph){

    srrg2_core::Chrono ch1("optimizeFactorGraph: ", &_timings, false);

    // Instanciate a solver
    srrg2_solver::Solver solver;
    // Remove default termination criteria
    solver.param_termination_criteria.setValue(nullptr);
    // Set max number of iterations
    solver.param_max_iterations.pushBack(1);
    // Change iteration algorithm to LM
    std::shared_ptr<srrg2_solver::IterationAlgorithmLM> lm(new srrg2_solver::IterationAlgorithmLM);
    lm->param_user_lambda_init.setValue(1e-7);
    solver.param_algorithm.setValue(lm);

    // Connect the graph to the solver and compute
    solver.setGraph(graph);
    // solver.saveGraph("before.graph");

    // Optimize the graph
    solver.compute();
    // solver.saveGraph("after.graph");
    // Visualize statistics and exit
    const auto& stats = solver.iterationStats();
    std::cout << "performed [" << FG_YELLOW(stats.size()) << "] iterations" << std::endl;
    std::cout << "stats\n\n";
    std::cout << stats << std::endl;

    // std::cout << "Diagonal " << solver.getDiagonal() << std::endl;
    // std::cout << "Block (0,0) " <<  
    solver.H().blockAt(0,0)->print() ;
    // << std::endl;

  }

  void PointCloudProc::addSurfelFactors(const srrg2_solver::FactorGraphPtr& graph) {
    // Iterate through all surfels
    // Add everything as VariableSE3QuaternionRight

    // Set lastID to the number of poses
    static int64_t lastGraphId = poses_.size() - 1;

    // For each surfel
    for (const std::shared_ptr<Surfel>& surfel : surfels_) {

      // Set containing ids of odom poses
      std::set<uint> odomPoseIdSet;

      // For each surfel's pose observation
      // if (surfel->odomPoses_.size() != 3)
        // continue;
        
      for (uint i = 0; i < surfel->odomPoses_.size(); i++) {


        // static int surfelCnt = 0;
        // if (surfelCnt++ > 10) {
        //   std::cout << "Surfel break" << std::endl;
        //   break;
        // }

        // Detect if given surfel has 2 or more observations from the same pose
        // TODO Check if removing this changes something
        uint odomPoseId = surfel->odomPosesIds_.at(i);
        // if (odomPoseIdSet.find(odomPoseId) != odomPoseIdSet.end())
          // continue;

        // Ad that pose id to a set
        odomPoseIdSet.insert(odomPoseId);

        // Create multiple subsurfel variables for each odomPoses_
        auto surfelVar = std::make_shared<srrg2_solver::VariableSE3QuaternionRight>();
        surfelVar->setGraphId(++lastGraphId);
        Eigen::Isometry3d surfelPose = Eigen::Isometry3d::Identity();
        surfelPose.translation() = surfel->observations_.at(i).block<3,1>(0,3);
        // surfelPose.linear() = surfel->observations_.at(i).block<3,3>(0,0);
        surfelPose.linear() =  matrixBetween2Vectors(Eigen::Vector3d(1, 0, 0), surfel->observations_.at(i).block<3,1>(0,0));
        // Connect subsurfels with poses and set estimates / measurements
        surfelVar->setEstimate(surfelPose.cast<float>());
        graph->addVariable(surfelVar);


        // Create factor between pose and the surfel
        // Eigen::Isometry3d surfelOdomPose = surfel->odomPoses_.at(i);
  
        auto poseSurfelFactor = std::make_shared<srrg2_solver::SE3PosePoseGeodesicErrorFactor>();
        poseSurfelFactor->setVariableId(0, (int64_t)odomPoseId);
        poseSurfelFactor->setVariableId(1, surfelVar->graphId());

        // poses_.at(odomPoseId) should be the same as surfel->odomPoses_(i)
        if (poses_.at(odomPoseId).matrix() != surfel->odomPoses_.at(i).matrix())
          std::cout << "Error: Poses stored in surfel and saved as odometetry don't match" << std::endl;

        Eigen::Isometry3f surfelInPose = (surfel->odomPoses_.at(i).inverse() * surfelPose).cast<float>();
        poseSurfelFactor->setMeasurement(surfelInPose);

        // Set information matrix
        Eigen::Matrix<float, 6, 6> infMat = Eigen::Matrix<float, 6, 6>::Identity();
        infMat *= 1000;
        poseSurfelFactor->setInformationMatrix(infMat);

        // Add factor to the graph
        graph->addFactor(poseSurfelFactor);

        // Connect subsurfels themselfes
        auto surfelSurfelFactor = std::make_shared<srrg2_solver::SE3Plane2PlaneErrorFactor>();
        surfelSurfelFactor->setVariableId(0, surfelVar->graphId()-1);
        // surfelSurfelFactor->setVariableId(1, surfelVar->graphId());

        // Eigen::Isometry3f surfelInSurfel = (surfel->odomPoses_.at(i).inverse() * surfelPose).cast<float>();
        // surfelSurfelFactor->setMeasurement(surfelInSurfel);

        // Set information matrix as Identity
        // Eigen::Matrix<float, 4, 4> infMat = Eigen::Matrix<float, 4, 4>::Identity();
        // surfelSurfelFactor->setInformationMatrix(infMat);

        // Add factor to the graph
        // graph->addFactor(surfelSurfelFactor);
      }
    }
  }

  void PointCloudProc::createPoseArrayfromGraph(geometry_msgs::PoseArray& poseArray, const srrg2_solver::FactorGraphPtr & graph) {
    // visualize the poses

    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "map";
    for (auto varIt : graph->variables()) {
      // srrg2_solver::VariableBase* v = varIt.second;
      srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(varIt.second);
      if (!varPose) {
        continue;
      }
      // Create new pose Msg
      geometry_msgs::Pose poseMsg;
      Eigen::Isometry3f pose = varPose->estimate();
      poseMsg.position.x = pose.translation().x();
      poseMsg.position.y = pose.translation().y();
      poseMsg.position.z = pose.translation().z();
      // Add orientation
      Eigen::Quaternionf quat(pose.linear());
      quat.normalize();
      poseMsg.orientation.x = quat.x();
      poseMsg.orientation.y = quat.y();
      poseMsg.orientation.z = quat.z();
      poseMsg.orientation.w = quat.w();
      // Add pose to poseArray
      poseArray.poses.push_back(poseMsg);
    }
  }

  void PointCloudProc::publishTFFromGraph(const srrg2_solver::FactorGraphPtr & graph) {
    // Republish the tf's
    int cnt = 0;
    for (auto varIt : graph->variables()) {
      // srrg2_solver::VariableBase* v = varIt.second;
      srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(varIt.second);
      if (!varPose) {
        continue;
      }
      srrg2_solver::VariableSE3QuaternionRight* varPose2 = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(cnt));
      if (!varPose->estimate().matrix().isApprox(varPose2->estimate().matrix())) {
        std::cout << "Error in publishTFFromGraph" << std::endl;
        exit(0);
      }
      geometry_msgs::TransformStamped tfStamped;
      tfStamped.header.stamp = ros::Time::now();
      tfStamped.header.frame_id = "map";
      tfStamped.child_frame_id = "ba_estimate_" + std::to_string(cnt++);

      Eigen::Isometry3f pose = varPose->estimate();
      Eigen::Quaternionf quat(pose.linear());
      quat.normalize();

      tfStamped.transform.translation.x = pose.translation().x();
      tfStamped.transform.translation.y = pose.translation().y();
      tfStamped.transform.translation.z = pose.translation().z();
      tfStamped.transform.rotation.x = quat.x();
      tfStamped.transform.rotation.y = quat.y();
      tfStamped.transform.rotation.z = quat.z();
      tfStamped.transform.rotation.w = quat.w();
      transformBroadcaster_.sendTransform(tfStamped);

      // Compare the pose with GT
      // std::cout << "GT pose  " << cnt-1 << std::endl << posesWithoutNoise_.at(cnt-1).matrix() << std::endl << "Actual Pose " << std::endl << pose.matrix() << std::endl;
    }
  }

    void PointCloudProc::publishSurfFromGraph(const srrg2_solver::FactorGraphPtr & graph) {
    // Republish the tf's
    int cnt = 0;
    for (auto varIt : graph->variables()) {
      // srrg2_solver::VariableBase* v = varIt.second;
      srrg2_solver::VariableSurfelAD1D* varSurf = dynamic_cast<srrg2_solver::VariableSurfelAD1D*>(varIt.second);
      if (!varSurf) {
        continue;
      }

      // std::cout << "SurfelPose: " << std::endl << varSurf->estimate().matrix() << std::endl;

      geometry_msgs::TransformStamped tfStamped;
      tfStamped.header.stamp = ros::Time::now();
      tfStamped.header.frame_id = "map";
      tfStamped.child_frame_id = "surfel_" + std::to_string(cnt);

      Eigen::Isometry3f pose = varSurf->estimate();
      Eigen::Quaternionf quat(pose.linear());
      quat.normalize();

      tfStamped.transform.translation.x = pose.translation().x();
      tfStamped.transform.translation.y = pose.translation().y();
      tfStamped.transform.translation.z = pose.translation().z();
      tfStamped.transform.rotation.x = quat.x();
      tfStamped.transform.rotation.y = quat.y();
      tfStamped.transform.rotation.z = quat.z();
      tfStamped.transform.rotation.w = quat.w();
      // transformBroadcaster_.sendTransform(tfStamped);
      static int markerColor = 0;
      // float radius = surfels_.at(cnt)->getLargestRadius();
      float radius = 0.2;
      float thickness = 0.05;
      visual_tools_->publishCylinder(varSurf->estimate().cast<double>(), static_cast<rviz_visual_tools::colors>(/* markerColor++ % 11 */ 7), thickness, radius);
      cnt++;
    }
    visual_tools_->trigger();
  }

  void PointCloudProc::updateSurfelsMeanFromGraph(const srrg2_solver::FactorGraphPtr & graph, std::vector<Surfelv2>& surfelsv2){
    int surfelIdx = 0;
    for (auto varIt : graph->variables()) {
      srrg2_solver::VariableSurfelAD1D* varSurf = dynamic_cast<srrg2_solver::VariableSurfelAD1D*>(varIt.second);
      if (!varSurf) {
        continue;
      }
      surfelsv2.at(surfelIdx++).setMeanEst(varSurf->estimate().translation());
    }
    if (surfelIdx != surfelsv2.size()) {
      std::cout << "Error in updateSurfelsMeanFromGraph" << std::endl;
      exit(0);
    }
  }

  void PointCloudProc::publishPointSurfv2(std::vector<Surfelv2>& surfelsv2) {
     
    pcl::PointSurfel ps;
    pcl::PointCloud<pcl::PointSurfel> psCloud;
    for (const Surfelv2 & surfel: surfelsv2){
      Eigen::Vector3f t = surfel.getMeanEst();
      Eigen::Vector3f n = surfel.getNormal();
      float radius = surfel.getMaxRadius();
      if (radius < 0.075)
        radius = 0.075;
      uint8_t r = 0, g = 255, b = 0, a = 255;
      pcl::PointSurfel ps;
      ps.x = t.x();
      ps.y = t.y();
      ps.z = t.z();
      ps.normal_x = n.x();
      ps.normal_y = n.y();
      ps.normal_z = n.z();
      ps.radius = radius;
      psCloud.push_back(ps);
    }
    // Convert to ROS data type
    
    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(psCloud, rosCloud);

    // Set the header
    rosCloud.header.stamp = ros::Time::now();
    rosCloud.header.frame_id = "map";

    surfelPointCloudPub_.publish(rosCloud);
  }

  void PointCloudProc::publishPointClouds() {
    for (sensor_msgs::PointCloud2& pc : rosPointClouds_) {
      pc.header.stamp = ros::Time::now();
      pointCloudPub_.publish(pc);
    }
  }

  void PointCloudProc::addNoiseToLastPose() {
    // Define the noise added to the poses
    static std::random_device rd;                                  // obtain a random number from hardware
    static std::mt19937 gen(rd());                                 // seed the generator
    static std::normal_distribution<double> noiseTrans(0.0, 0.3);  // define the noise for translation
    static std::normal_distribution<double> noiseRot(0.0, 3.0 * M_PI / 180.0);   // define the noise for rotation

    // Take all poses
    Eigen::Isometry3d perturbation = Eigen::Isometry3d::Identity();
    // for (Eigen::Isometry3d& pose : poses_.back()) {
    Eigen::Isometry3d& pose = poses_.back();

                              // Generate the noise matrix
                              perturbation.translation() = Eigen::Vector3d(noiseTrans(gen), noiseTrans(gen), noiseTrans(gen));
    perturbation.linear() = (Eigen::AngleAxisd(noiseRot(gen), Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(noiseRot(gen), Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(noiseRot(gen), Eigen::Vector3d::UnitZ()))
                                .toRotationMatrix();

    // Add noise to the pose
    Eigen::Isometry3d constPerturbation = Eigen::Isometry3d::Identity();
    constPerturbation.translation() = Eigen::Vector3d(0.6, 0.8, 0.9);
    constPerturbation.linear() = (Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()))
                                     .toRotationMatrix();

    // Save pose without noise
    posesWithoutNoise_.push_back(pose);
    static int cnt = 0;
    // Do not add noise to the first pose? Sometimes usefull
    if (cnt++ != 0)
      pose = pose * perturbation;

    // Add noise only to the second pose
    // static int cnt = 0;
    // if (cnt++ == 1)
      // pose = constPerturbation * pose;
    // std::cout << "Pose with noise  " << pose.matrix() << std::endl;
    // }
  }

  }  // namespace structure_refinement
