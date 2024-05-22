#include "point_cloud_proc.h"

namespace structure_refinement {
  using namespace srrg2_core;
  using namespace srrg2_core_ros;
  using LidarProjectorType   = PointIntensity3fProjectorOS1_64;
  using LidarUnprojectorType = PointIntensity3fUnprojectorOS1_64;
  using NormalComputatorType = NormalComputator2DCrossProduct<PointNormal3fVectorCloud,0>; //NormalComputator2DCrossProduct<PointNormal3fMatrixCloud, 1>;

  PointCloudProc::PointCloudProc() : nh_("~") {

      // ROS publishers
      pointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw_point_cloud", 100);
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
      // Saves some RAM if false (4GB for ~2000 poses)
      visualizePointClouds_ = false; 
      // Whether to save scans to output/scans
      saveSurfelsScans_ = false
      // Skip first n messages and process only m first clouds
      int cloudsToSkip = 0;
      int decimateRealData = 1;
      int cloudsToProcess = 1990 * decimateRealData;
      int iterNum = 3;
      static int msgCnt = -1;
      if (++msgCnt < 2 * cloudsToSkip) {
        return true;
      } else if (msgCnt > 2 * (cloudsToSkip + cloudsToProcess) - 2) {
        for (int i = 0; i < iterNum; i++) {
          DataAssociation dataAssociation;
          {
            // srrg2_core::Chrono chGP("prepareData GPU", &_timings, false);
            // dataAssociation.prepareData(kdTrees_, kdTreeLeafes_);
          }

          {
            srrg2_core::Chrono chGP2("prepareData CPU", &_timings, false);
            dataAssociation.prepareDataCPU(kdTrees_, kdTreeLeafes_);
            // Remove surfels with only 1-point leafs
            dataAssociation.filterSurfels();
          }

          // Publish and save to file
          publishPointSurfv2(dataAssociation.getSurfels());
          savePosesToFile();

          handleFactorGraph(dataAssociation.getSurfels());
          // visual_tools_->deleteAllMarkers();
          // visualizeCorrespondingSurfelsV2WithPoses(dataAssociation.getSurfels());

          // if last iteraton publish and save to file
          if (i == iterNum - 1) {
            publishPointSurfv2(dataAssociation.getSurfels());
            savePosesToFile();
            if (saveSurfelsScans_)
              createAndSaveScans(dataAssociation.getSurfels());
          }

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

  void PointCloudProc::savePosesToFile(){
    static std::string path = ros::package::getPath("structure_refinement") + "/output/tum/";
    static int cnt = 0;
    std::string filename = "optimized_trajectory_" + std::to_string(cnt++) +  ".tum";
    std::fstream file;
    file.open(path + filename, std::fstream::out);  // Creates empty file
    for (int i = 0; i < poses_.size(); i++) {
      Eigen::Isometry3f pose = poses_.at(i);
      Eigen::Quaternionf quat(pose.linear());
      file << posesTimestamps_.at(i) << " "
           << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " " 
           << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
    }
    file.close();
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
      Eigen::Vector3f trans(rosMsgPtr->pose.pose.position.x, rosMsgPtr->pose.pose.position.y, rosMsgPtr->pose.pose.position.z);
      Eigen::Quaternionf quat(rosMsgPtr->pose.pose.orientation.w, rosMsgPtr->pose.pose.orientation.x, rosMsgPtr->pose.pose.orientation.y, rosMsgPtr->pose.pose.orientation.z);
      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      pose.translation() = trans;
      pose.linear() = quat.toRotationMatrix();
      poses_.push_back(pose);
      posesTimestamps_.push_back(rosMsgPtr->header.stamp);

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
          static int idx_cloud = 0;
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

          if (visualizePointClouds_) {
            // Create the point cloud to set new intensity value
            PointIntensity3fVectorCloud pointCloudI3f;

            // Get the point from original point cloud
            cloud->getPointCloud(pointCloudI3f);

            // Set the Intensity for Rviz visualization
            for (PointIntensity3f& p : pointCloudI3f) {
              p.intensity() = (float)cloudNum;
            }
            // Set point to the visualizaton point cloud - overrides the intensity of the original cloud
            cloud->setPointCloud(pointCloudI3f);

            // Convert srrg2 to sensor_msgs point cloud
            sensor_msgs::PointCloud2ConstPtr rosMsg = Converter::convert(cloud);

            // Store the point cloud to visualize it after optimization etc.
            rosPointClouds_.push_back(*rosMsg);
          }

          // Publish the raw cloud as ROS message
        //   pointCloudPub_.publish(*rosMsg);

          // Create Point3f Point Cloud from srrg2 point cloud
          Point3fVectorCloud pointCloudPoint3f; //(new Point3fVectorCloud());
          cloud->getPointCloud(pointCloudPoint3f);

          // TODO: optimize
          // Conver point cloud to the std::vector<Eigen::Vector3f
          std::vector<Eigen::Vector3f> pointCloudEigen;
          // For now copy all the points from Point3f point cloud to std::vector<Eigen::Vector3f>
          for (const Point3f& p : pointCloudPoint3f) {
              Eigen::Vector3f pe(p.coordinates());
              pointCloudEigen.push_back(pe);
          }

          // For each point cloud create an KDTree and leafs | Get the last pose 
          createKDTree(pointCloudEigen, poses_.at(idx_cloud));
          // Publish the normals created from the last point cloud
          //   int idx = kdTreeLeafes_.size()-1;
          //   publishCloudNormals(idx);

          // Store Point3f cloud in a vector
          // pointClouds_.push_back(std::move(pointCloudPoint3f));
          idx_cloud++;
          // Remove cloud from buffer
          cloudBuffer.erase(cloudBuffer.begin());
      }
  }



  void PointCloudProc::publishCloudNormals(int idx) {
      // Delete all markers from Rviz
        // visual_tools_->deleteAllMarkers();

      static uint8_t markerColor = 0;  // Colors change <0,14>

      uint8_t decimation = 1;

      // Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      for (auto& leaf : kdTreeLeafes_.at(idx)) {
        // Decimate for Rviz
        if (static int cnt = 0; cnt++ % decimation != 0)
          continue;
        if (++markerColor > 14)
          markerColor = 0;

        visualizeSurfel(leaf, markerColor);
      }
  }


  void PointCloudProc::createKDTree(std::vector<Eigen::Vector3f>& cloud, const Eigen::Isometry3f& lastPose) {
      using ContainerType = std::vector<Eigen::Vector3f>;
      using TreeNodeType = TreeNode3D<ContainerType>;
      using TreeNodeTypePtr = TreeNodeType*;

      static int pointCloudId = -1;
      pointCloudId++;
      // Create kdTree from the point cloud
      std::unique_ptr<TreeNodeType> kdTree = std::make_unique<TreeNodeType>(cloud.begin(),
                                              cloud.end(),
                                              0.2,
                                              0.1,
                                              0,
                                              2,
                                              nullptr,
                                              nullptr,
                                              pointCloudId);

      // Transform kdTree according to the BA pose
      // Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
      // Eigen::Vector3f trans = Eigen::Vector3f(20,20,20);
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

      Eigen::Isometry3f poseInc = Eigen::Isometry3f::Identity();
      poseInc.translation() = Eigen::Vector3f(1.0, 1.0, 0.01);
      float roll = 0.01 * 3, pitch = 0.03 * 2, yaw = 0.05;
      poseInc.linear() = (Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
                          Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                          Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()))
                             .toRotationMatrix();

      // Change inc direction after 5 poses
      static int cnt = 0;
      if (cnt++ > 5)
        poseInc.translation().x() *= -1;

      // Calculate new pose
      Eigen::Isometry3f newPose = Eigen::Isometry3f::Identity();
      if (poses_.size() != 0)
          newPose = poses_.back() * poseInc;
      else{
          // Initial pose
          newPose.translation() = Eigen::Vector3f(3, 2, 2);
          newPose.linear() = poseInc.linear();
      }

      // Fill up the odometry message
      Eigen::Quaternionf quat(newPose.linear());

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
      static std::normal_distribution<float> noise(0, 0.03);  // noise (mean, var)

      // Synthethic cloud
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> pointCloudSynth;

      // Room dimensons
      float width = 10, depth = 15, height = 5;
      float pointsDensity = 5;  // Pts per meter
      Eigen::Vector3f point;
      // Generate floor
      for (int i = 0; i <= width * pointsDensity; i++) {
          for (int j = 0; j <= depth * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3f(i / pointsDensity + noise(gen), j / pointsDensity + noise(gen), 0 + noise(gen)));
          }
      }
      // Generate front and back walls
      for (int i = 0; i <= width * pointsDensity; i++) {
          for (int j = 1; j < height * pointsDensity; j++) {
              pointCloudSynth.push_back(Eigen::Vector3f(i / pointsDensity + noise(gen), 0 + noise(gen), j / pointsDensity + noise(gen)));
              pointCloudSynth.push_back(Eigen::Vector3f(i / pointsDensity + noise(gen), depth + noise(gen), j / pointsDensity + noise(gen)));
          }
      }

      // Generate left and right walls
      for (int i = 1; i < depth * pointsDensity; i++) {
          for (int j = 1; j < height * pointsDensity; j++) {
            pointCloudSynth.push_back(Eigen::Vector3f(0 + noise(gen), i / pointsDensity + noise(gen), j / pointsDensity + noise(gen)));
            pointCloudSynth.push_back(Eigen::Vector3f(width + noise(gen), i / pointsDensity + noise(gen), j / pointsDensity + noise(gen)));
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
      Eigen::Isometry3f & lastSynthPose = poses_.back();
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
    static std::normal_distribution<float> noise(0.1, 0.2);  //  // define the range

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


  void PointCloudProc::handleFactorGraph(std::vector<Surfelv2>& surfelsv2) {
    // Create graph variable
    srrg2_solver::FactorGraphPtr graph(new srrg2_solver::FactorGraph);
    {
      srrg2_core::Chrono chGP2("Adding poses to graph", &_timings, false);
      addPosesToGraphBA(graph, poses_);
    }

      publishTFFromGraph(graph);
    if (visualizePointClouds_){
      publishPointClouds();
      publishTFFromGraph(graph);
    }

    // Add surfels
    {
      srrg2_core::Chrono chGP2("Adding surfels to graph", &_timings, false);
      addSurfelsToGraph(graph, surfelsv2);
    }
    // visual_tools_->deleteAllMarkers();

    // publishSurfFromGraph(graph);


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
    if(visualizePointClouds_) {
      publishPointClouds();
      publishTFFromGraph(graph);
    }
    // Visualize surfels
    // visual_tools_->deleteAllMarkers();
    // publishSurfFromGraph(graph);
    graph->clear();
  }

  void PointCloudProc::updatePosesAndLeafsFromGraph(srrg2_solver::FactorGraphPtr& graph) {
    posesInGraph_.clear();
    for (uint i = 0; i < poses_.size(); i++) {
      srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(i));
      if (!varPose) {
        std::cout << "Error in updatePosesInGraph" << std::endl;
        exit(0);
      }
      Eigen::Isometry3f poseInc = varPose->estimate().cast<float>() * poses_.at(i).inverse();
      if(i < kdTrees_.size())
        kdTrees_.at(i)->applyTransform(poseInc.linear(), poseInc.translation());
      poses_.at(i) = varPose->estimate().cast<float>();
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
    posesInGraph_.push_back(varPose->estimate().cast<float>());

  }
}

std::vector<Eigen::Isometry3f> PointCloudProc::addSynthSurfelsToGraphBA(srrg2_solver::FactorGraphPtr& graph, std::vector<SynthSurfel> & surfOut) {

  int64_t lastGraphId = poses_.size() - 1;

  std::vector<Eigen::Isometry3f> surfelGTVector;

  // Used to generate random position of a surfel
  Eigen::Isometry3f surfelGT = Eigen::Isometry3f::Identity();
  static std::random_device rdS;                            // obtain a random number from hardware
  static std::mt19937 genS(rdS());                          // seed the generator
  static std::normal_distribution<float> trans(5.0, 8.0);  // define the noise for translation
  static std::normal_distribution<float> rot(0.0, M_PI);    // define the noise for rotation

  // Generate the surfels GT positions vector
  for (int j = 0; j < 20; j++) {

    // Random position of a surfel
    surfelGT.translation() = Eigen::Vector3f(trans(genS), trans(genS), trans(genS));
    surfelGT.linear() = (Eigen::AngleAxisf(rot(genS), Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(rot(genS), Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(rot(genS), Eigen::Vector3f::UnitZ()))
                            .toRotationMatrix();
    surfelGTVector.push_back(surfelGT);
  }

  for (int j = 0; j < surfelGTVector.size(); j++) {
    auto surfelVar = std::make_shared<srrg2_solver::VariableSurfelAD1D>();
    Eigen::Isometry3f surfelPoseGT = surfelGTVector.at(j);
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
    //   visual_tools_->publishAxis(surfelInMap.cast<float>(), rviz_visual_tools::LARGE);
    //   visual_tools_->publishLine(varPose->estimate().translation().cast<float>(), surfelInMap.translation().cast<float>(), static_cast<rviz_visual_tools::colors>(markerColor%14));
      visual_tools_->trigger();

      // Add observation to synth surfel
       Eigen::Matrix<float,3,2> obs;
        // Position of a surfel
      obs.block<3,1>(0,0) = surfelInMap.translation().cast<float>();
        // Normal of a surfel
      obs.block<3,1>(0,1) = surfelInMap.linear().col(2).cast<float>();
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
        Eigen::Matrix<float,3,2> obs;
        synthSurfel.estPosition_ = surfelVar->estimate().translation().cast<float>();
        synthSurfel.estNormal_ = surfelVar->estimate().linear().col(2).cast<float>();
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


void PointCloudProc::addSurfelsToGraph(srrg2_solver::FactorGraphPtr& graph, std::vector<Surfelv2>& surfelsv2){

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
      Eigen::Isometry3f surfelPose = Eigen::Isometry3f::Identity();
      Eigen::Isometry3f rotationY = Eigen::Isometry3f::Identity();
      rotationY.linear() << 0, 0, 1, 0, 1, 0, -1, 0, 0;
      surfelPose.translation() = surfel.leafs_.at(0)->mean_;
      surfelPose.linear() = matrixBetween2Vectors(Eigen::Vector3f(0, 0, 1), surfel.leafs_.at(0)->eigenvectors_.col(0));
      surfelVar->setEstimate(surfelPose.cast<float>());

      // std::cout << "SurfelPose before optim: " << std::endl << surfelVar->estimate().matrix() << std::endl;
      // surfelVar->setStatus(srrg2_solver::VariableBase::Status::Fixed);

      graph->addVariable(surfelVar);

      // Iterate through all poses from which given surfel was registered
      for (uint i = 0; i < surfel.leafs_.size(); i++) {

        // Create factor between pose and the surfel
        // Eigen::Isometry3f surfelOdomPose = surfel->odomPoses_.at(i);
        std::shared_ptr<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D> poseSurfelFactor = std::make_shared<srrg2_solver::SE3PoseSurfelQuaternionErrorFactorAD1D>();
        poseSurfelFactor->setVariableId(0, surfel.leafs_.at(i)->pointcloud_id_);
        poseSurfelFactor->setVariableId(1, surfelVar->graphId());
        // poses_ .at(odomPoseId) should be the same as surfel->odomPoses_(i)
        // if (posesInGraph_.at(odomPoseId).matrix() != surfel->odomPoses_.at(i).matrix())
          // std::cout << "Error: Poses stored in surfel and saved as odometetry don't match" << std::endl;

        Eigen::Isometry3f odomPose = poses_.at(surfel.leafs_.at(i)->pointcloud_id_).cast<float>();
        Eigen::Isometry3f surfelInMap = Eigen::Isometry3f::Identity();
        surfelInMap.translation() = surfel.leafs_.at(i)->mean_.cast<float>();
        surfelInMap.linear() = matrixBetween2Vectors(Eigen::Vector3f(0, 0, 1), surfel.leafs_.at(0)->eigenvectors_.col(0)).cast<float>();
        Eigen::Isometry3f surfInPose = odomPose.inverse() * surfelInMap;
        poseSurfelFactor->setMeasurement(surfInPose);

        // Calculate angle inclination
        Eigen::Vector3f surfNormal = surfel.leafs_.at(0)->eigenvectors_.col(0).cast<float>();
        float cosa = surfNormal.dot(surfInPose.translation()) / (surfNormal.norm() * surfInPose.translation().norm());
        // Set information matrix
        Eigen::Matrix<float, 1, 1> infMat = Eigen::Matrix<float, 1, 1>::Identity();
        infMat *= abs(cosa);
        poseSurfelFactor->setInformationMatrix(infMat);
        // poseSurfelFactor->setInformationMatrix()

        // Add factor to the graph
        graph->addFactor(poseSurfelFactor);
      }
    }
}

void PointCloudProc::addPosesToGraphBA(srrg2_solver::FactorGraphPtr& graph, std::vector<Eigen::Isometry3f>& poseVect) {

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


void PointCloudProc::updateLeafsPosition(srrg2_solver::FactorGraphPtr& graph, std::vector<Eigen::Isometry3f> & lastPosesInGraph) {
  for (uint i = 0; i < kdTrees_.size(); i++) {
    srrg2_solver::VariableSE3QuaternionRight* varPose = dynamic_cast<srrg2_solver::VariableSE3QuaternionRight*>(graph->variable(i));
    if (!varPose) {
      continue;
    }
    // std::cout << "Id of variable   "  << varPose->graphId() << std::endl;
    Eigen::Isometry3f poseInc = varPose->estimate().cast<float>() * poses_.at(i).inverse() ;
    // poseInc.matrix() *= 0.1;
    // std::cout << "Change in pose #" << i << std::endl << poseInc.matrix() << std::endl;
    // It's enough to transform only kdTree_, as kdTreeLeafes_ contains pointers to them
    // varPose->estimate() * lastPoseInGraph.inverse() * 
    kdTrees_.at(i)->applyTransform(poseInc.linear(), poseInc.translation());  
  }
}

void PointCloudProc::rawOptimizeSynthSurfels(const std::vector<SynthSurfel> & surfelsIn, std::vector<SynthSurfel> & surfelsOut){

  // std::vector<Eigen::Isometry3f> surfelsOut;
  // surfelsOut = surfelsIn;
  srrg2_core::Chrono ch1("rawOptimizeSynthSurfels: ", &_timings, false);

  // Iterate through all surfels
  for (auto s : surfelsIn) {
    float A = 0;
    float b = 0;
    for (int i = 0; i < s.observations_.size(); i++) {
      Eigen::Vector3f measPos = s.observations_.at(i).block<3, 1>(0, 0);
      Eigen::Vector3f measNorm = s.observations_.at(i).block<3, 1>(0, 1);

      float e = s.estNormal_.dot(s.estPosition_ - measPos);
      A += 1;
      b += 1 * e;     
    }
    float dx = -b / A;
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

  // std::vector<Eigen::Isometry3f> surfelsOut;
  // surfelsOut = surfelsIn;
  srrg2_core::Chrono ch1("rawOptimizeSurfels: ", &_timings, false);

  // Iterate through all surfels
  for (auto s : surfelsIn) {
    float A = 0;
    float b = 0;
    // Set the initial estimates to the first observations - the same as in the case of using solver
    s->estPosition_ = s->observations_.at(0).block<3, 1>(0, 3);
     s->estNormal_ = s->observations_.at(0).col(0); // block<3, 1>(0, 1);
    // s.estNormal_ = s.observations_.at(0).block<3, 1>(0, 1);

    for (int i = 0; i < s->observations_.size(); i++) {
      Eigen::Vector3f measPos = s->observations_.at(i).block<3, 1>(0, 3);
      Eigen::Vector3f measNorm = s->observations_.at(i).block<3, 1>(0, 0);

      float e = s->estNormal_.dot(s->estPosition_ - measPos);
      A += 1;
      b += 1 * e;     
    }
    float dx = -b / A;
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
        Eigen::Isometry3f surfelPose = Eigen::Isometry3f::Identity();
        surfelPose.translation() = surfel->observations_.at(i).block<3,1>(0,3);
        // surfelPose.linear() = surfel->observations_.at(i).block<3,3>(0,0);
        surfelPose.linear() =  matrixBetween2Vectors(Eigen::Vector3f(1, 0, 0), surfel->observations_.at(i).block<3,1>(0,0));
        // Connect subsurfels with poses and set estimates / measurements
        surfelVar->setEstimate(surfelPose.cast<float>());
        graph->addVariable(surfelVar);


        // Create factor between pose and the surfel
        // Eigen::Isometry3f surfelOdomPose = surfel->odomPoses_.at(i);
  
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
      r = rand() % 255;
      g = rand() % 255;
      b = rand() % 255;
      pcl::PointSurfel ps;
      ps.x = t.x();
      ps.y = t.y();
      ps.z = t.z();
      ps.normal_x = n.x();
      ps.normal_y = n.y();
      ps.normal_z = n.z();
      ps.r = r;
      ps.g = g;
      ps.b = b;
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

    static std::string path = ros::package::getPath("structure_refinement") + "/output/";
    static int cnt = 0;
    pcl::io::savePLYFile(path + "ply/surfelCloud_" + std::to_string(cnt) + ".ply", psCloud);
    pcl::io::savePCDFile(path + "pcd/surfelCloud_" + std::to_string(cnt) + ".pcd", psCloud);
    cnt++;
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
    static std::normal_distribution<float> noiseTrans(0.0, 0.0);  // define the noise for translation
    static std::normal_distribution<float> noiseRot(0.0, 0.0 * M_PI / 180.0);   // define the noise for rotation

    // Take all poses
    Eigen::Isometry3f perturbation = Eigen::Isometry3f::Identity();
    // for (Eigen::Isometry3f& pose : poses_.back()) {
    Eigen::Isometry3f& pose = poses_.back();

                              // Generate the noise matrix
                              perturbation.translation() = Eigen::Vector3f(noiseTrans(gen), noiseTrans(gen), noiseTrans(gen));
    perturbation.linear() = (Eigen::AngleAxisf(noiseRot(gen), Eigen::Vector3f::UnitX()) *
                             Eigen::AngleAxisf(noiseRot(gen), Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(noiseRot(gen), Eigen::Vector3f::UnitZ()))
                                .toRotationMatrix();

    // Add noise to the pose
    Eigen::Isometry3f constPerturbation = Eigen::Isometry3f::Identity();
    constPerturbation.translation() = Eigen::Vector3f(0.6, 0.8, 0.9);
    constPerturbation.linear() = (Eigen::AngleAxisf(5.0 * M_PI / 180.0, Eigen::Vector3f::UnitX()) *
                                  Eigen::AngleAxisf(5.0 * M_PI / 180.0, Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(5.0 * M_PI / 180.0, Eigen::Vector3f::UnitZ()))
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

  void PointCloudProc::createAndSaveScans(std::vector<Surfelv2>& surfelsv2){

    std::vector<pcl::PointCloud<pcl::PointSurfel>> psCloudsVec(poses_.size());

    for (const Surfelv2& surfel : surfelsv2) {

      // Iterate through all poses from which given surfel was registered
      for (uint i = 0; i < surfel.leafs_.size(); i++) {
        Eigen::Isometry3f poseInv = poses_.at(surfel.leafs_.at(i)->pointcloud_id_).inverse();
        Eigen::Vector3f t = surfel.getMeanEst();
        Eigen::Vector3f n = surfel.getNormal();
        t = poseInv.linear() * t + poseInv.translation();
        n = poseInv.linear() * n;
        float radius = surfel.getMaxRadius();
        if (radius < 0.075)
          radius = 0.075;
        uint8_t r = 255, g = 255, b = 255, a = 255;
        pcl::PointSurfel ps;
        ps.x = t.x();
        ps.y = t.y();
        ps.z = t.z();
        ps.normal_x = n.x();
        ps.normal_y = n.y();
        ps.normal_z = n.z();
        ps.radius = radius;
        ps.r = r;
        ps.g = g;
        ps.b = b;
        psCloudsVec.at(surfel.leafs_.at(i)->pointcloud_id_).push_back(ps);
      }
    }

    // Save scans to file and bag
    rosbag::Bag bag;
    std::string path = ros::package::getPath("structure_refinement") + "/output/";
    bag.open(path + "scans/bag/test.bag", rosbag::bagmode::Write);
    int scanCnt = 0;
    for (auto & psCloud: psCloudsVec){
      if (psCloud.size() == 0)
        continue;
      pcl::io::savePLYFile(path + "scans/ply/surfelScan_" + std::to_string(scanCnt) + ".ply", psCloud);
      pcl::io::savePCDFile(path + "scans/pcd/surfelScan_" + std::to_string(scanCnt) + ".pcd", psCloud);

      sensor_msgs::PointCloud2 rosCloud;
      pcl::toROSMsg(psCloud, rosCloud);
      // Set the header
      rosCloud.header.stamp = posesTimestamps_.at(scanCnt);
      rosCloud.header.frame_id = "map";
      bag.write("surfelScan", posesTimestamps_.at(scanCnt), rosCloud);
      scanCnt++;
    }
  }
}  // namespace structure_refinement
