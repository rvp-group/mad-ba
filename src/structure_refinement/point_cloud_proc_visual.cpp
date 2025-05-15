#include "point_cloud_proc.h"

namespace mad_ba {
using namespace srrg2_core;
using namespace srrg2_core_ros;
using LidarProjectorType = PointIntensity3fProjectorOS1_64;
using LidarUnprojectorType = PointIntensity3fUnprojectorOS1_64;
using NormalComputatorType = NormalComputator2DCrossProduct<PointNormal3fVectorCloud, 0>;  // NormalComputator2DCrossProduct<PointNormal3fMatrixCloud, 1>;

void PointCloudProc::visializeAllSurfels() {
  Eigen::Isometry3f surfelPose = Eigen::Isometry3f::Identity();
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
      surfelPose.translation() = Eigen::Vector3f(surfel->mean_);
      // Calculate the rotation matrix
      Eigen::Matrix3f rotMatrix = matrixBetween2Vectors(Eigen::Vector3f(1, 0, 0), surfel->eigenvectors_.col(0));
      surfelPose.linear() = rotMatrix;
      // Publish normal as arrow
      visual_tools_->publishArrow(surfelPose.cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::XXXLARGE);
    }
  }
  visual_tools_->trigger();
}

void PointCloudProc::visializeSurfelsv2(std::vector<Surfelv2>& surfelsv2) {
  Eigen::Isometry3f surfelPose = Eigen::Isometry3f::Identity();
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

    if (surfelCnt > 500 * decimation)
      break;
    // Get the surfels
    for (auto& leaf : surfel.leafs_) {
      // Decimate for Rviz
      // if (leafCnt++ % decimation != 0)
      //   continue;

      // Set the translation part
      surfelPose.translation() = Eigen::Vector3f(leaf->mean_);
      // Calculate the rotation matrix
      Eigen::Matrix3f rotMatrix = matrixBetween2Vectors(Eigen::Vector3f(1, 0, 0), leaf->eigenvectors_.col(0));
      surfelPose.linear() = rotMatrix;
      // Publish normal as arrow
      visual_tools_->publishArrow(surfelPose.cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::XXXLARGE);
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
    for (const Eigen::Isometry3f& pose : surfel->odomPoses_) {
      // Create new pose Msg
      geometry_msgs::Pose poseMsg;
      poseMsg.position.x = pose.translation().x();
      poseMsg.position.y = pose.translation().y();
      poseMsg.position.z = pose.translation().z();
      // Add orientation
      Eigen::Quaternionf quat(pose.linear());
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

  unsigned int surfelsToVisualize = 10;

  // Create vector containing sorted indices of surfels
  std::vector<int> idxSorted(surfelsv2.size());
  std::size_t n(0);
  std::generate(std::begin(idxSorted), std::end(idxSorted), [&] { return n++; });
  std::sort(std::begin(idxSorted), std::end(idxSorted), [&](int i1, int i2) { return surfelsv2[i1].leafs_.size() > surfelsv2[i2].leafs_.size(); });

  std::cout << "maxleafsId:  " << maxLeafsId << "  first el: " << idxSorted[0] << std::endl;

  // for (unsigned int i = 0; i < surfelsv2.size(); i += decimation) {
  static int markerColor = 9;
  for (unsigned int i = 0; i < surfelsToVisualize; i++) {
    markerColor = ++markerColor % 14;
    for (auto const& leaf : surfelsv2.at(idxSorted[i]).leafs_) {
      static int visCnt = 0;
      visualizeSurfel(leaf, markerColor);
      Eigen::Isometry3f trans = Eigen::Isometry3f::Identity();
      trans = poses_.at(leaf->pointcloud_id_);
      visual_tools_->publishLine(trans.translation().cast<double>(), leaf->mean_.cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::LARGE);
      visual_tools_->trigger();
    }
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
  for (unsigned int i = 0; i < surfels_.size(); i += decimation) {
    //   for (const Eigen::Isometry3f& pose : surfels_.at(i)->poses_) {
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
          // Eigen::Isometry3f trans = surfels_.at(i)->odomPoses_.at(poseId.first);
          Eigen::Isometry3f trans = Eigen::Isometry3f::Identity();
          if (posesInGraph_.size() == 0)
            trans = poses_.at(poseId.first);
          else
            trans = posesInGraph_.at(poseId.first);

          visual_tools_->publishLine(trans.translation().cast<double>(), leaf->mean_.cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor % 14), rviz_visual_tools::LARGE);
          visual_tools_->trigger();
        }
      }
    }
  }
}

void PointCloudProc::visualizeSurfel(TreeNodeType* surfel, int markerColor) {
  Eigen::Isometry3f surfelPose = Eigen::Isometry3f::Identity();
  surfelPose.translation() = Eigen::Vector3f(surfel->mean_);
  // Works even for surfels that have only 1 point (they have only the first column of egigenvector matrix)
  surfelPose.linear() = matrixBetween2Vectors(Eigen::Vector3f(1, 0, 0), surfel->eigenvectors_.col(0));
  // Get the surfel bounding box
  Eigen::Vector3f bbox = surfel->bbox_;
  for (int i = 0; i < 3; i++)
    if (bbox[i] < 0.01)
      bbox[i] = 0.01;

  // Publish normal as Arrow
  visual_tools_->publishArrow(surfelPose.cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::MEDIUM);

  // Publish surfel as Cylinder
  // Get the rotation matrix for 90* in Y axis
  Eigen::Matrix3f rotationY;
  rotationY << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  surfelPose.linear() = surfelPose.linear() * rotationY;
  float radius = (bbox[1] + bbox[2]) / 2.0;
  visual_tools_->publishCylinder(surfelPose.cast<double>(), static_cast<rviz_visual_tools::colors>(markerColor), bbox[0], radius);

  // Check if eigenvectors contains NaNs
  if (surfel->eigenvectors_ == surfel->eigenvectors_) {
    // Eigen::Vector3f deltaX = surfel->eigenvectors_.col(0) * bbox[0];
    // Eigen::Vector3f deltaY = surfel->eigenvectors_.col(1) * bbox[1];
    // Eigen::Vector3f deltaZ = surfel->eigenvectors_.col(2) * bbox[2];

    // Publish the bound box diagonals
    // visual_tools_->publishLine(surfel->mean_ - deltaX / 2.0, surfel->mean_ + deltaX / 2.0, static_cast<rviz_visual_tools::colors>(markerColor % 14));
    // visual_tools_->publishLine(surfel->mean_ - deltaY / 2.0, surfel->mean_ + deltaY / 2.0, static_cast<rviz_visual_tools::colors>(markerColor % 14));
    // visual_tools_->publishLine(surfel->mean_ - deltaZ / 2.0, surfel->mean_ + deltaZ / 2.0, static_cast<rviz_visual_tools::colors>(markerColor % 14));

    // Publish surfel as rectangle
    surfelPose.linear() = surfel->eigenvectors_ * rotationY;
    visual_tools_->publishWireframeRectangle(surfelPose.cast<double>(), bbox[1], bbox[2], static_cast<rviz_visual_tools::colors>(markerColor), rviz_visual_tools::XLARGE);
  }
  visual_tools_->trigger();
}

}  // namespace mad_ba
