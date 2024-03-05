#pragma once

#include <srrg_config/configurable.h>
#include <srrg_config/property_configurable.h>
#include <srrg_property/property_eigen.h>
#include <srrg_property/property_vector.h>

#include <srrg_data_structures/matrix.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_pcl/normal_computator.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/active_drawable.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_ldl.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include <srrg_solver/solver_core/iteration_algorithm_lm.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "kdtree.hpp"
#include <random>
#include "surfel.h"
#include "json.hpp"


namespace structure_refinement {
  using namespace srrg2_core;
    using ContainerType = std::vector<Eigen::Vector3d>;
      using TreeNodeType = TreeNode3D<ContainerType>;
      using TreeNodeTypePtr = TreeNodeType*;

  class PointCloudProc : public srrg2_core::MessageSinkBase, public srrg2_core::ActiveDrawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyFloat, radius, "radius", 2.f, 0);
    PointCloudProc();
    virtual ~PointCloudProc();
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg) override;   // Handle messages from the srrg pipeline
    void handleCloudMessage(PointCloud2MessagePtr);                   // Handle the point cloud message
    void handleOdometryMessage(OdometryMessagePtr);                   // Handle the odometry pose message
    void handleTFMessage(TransformEventsMessagePtr);                   // Handle the tf message
    bool createIntensityImage(srrg2_core::BaseSensorMessagePtr msg);  // Just for tests
    void publishCloudNormals(int);                                         // Publish normals for n-th set of point cloud / pose
    void createKDTree(std::vector<Eigen::Vector3d> &cloud, const Eigen::Isometry3d &lastPose);  // Creates kd-trees from the vector of points
    Eigen::Matrix3d matrixBetween2Vectors(Eigen::Vector3d, Eigen::Vector3d );
    double angleBetween2Vectors(const Eigen::Vector3d &, const Eigen::Vector3d &);
    void mergeSurfels();
    void visualizeSurfel(TreeNodeTypePtr, int);  // ToDo: rename for leafs
    void visializeAllSurfels();
    void visualizeSurfelPoses();
    void visualizeCorrespondingSurfelsWithPoses();
    void saveSurfelsTofile();

    int findLeafId(unsigned int, TreeNodeTypePtr);  // Find the id of a leaf in a given kdTree
    void generateSyntheticPointCloud(sensor_msgs::PointCloud2 &);
    void generateSyntheticOdometry(nav_msgs::Odometry &);
    void createFactorGraph(srrg2_solver::FactorGraphPtr &);
    void createPoseArrayfromGraph(geometry_msgs::PoseArray &, const srrg2_solver::FactorGraphPtr &);
    void publishTFFromGraph(const srrg2_solver::FactorGraphPtr &);
    void publishPointClouds();
    void handleFactorGraph();
    void optimizeFactorGraph(srrg2_solver::FactorGraphPtr &);

   protected:
    using PointUnprojectorBase = srrg2_core::PointUnprojectorBase_<srrg2_core::PointNormalIntensity3fVectorCloud>;
    using PointUnprojectorPinhole = srrg2_core::PointUnprojectorPinhole_<srrg2_core::PointNormalIntensity3fVectorCloud>;
    using PointUnprojectorPolar = srrg2_core::PointUnprojectorPolar_<srrg2_core::PointNormalIntensity3fVectorCloud>;

    std::unique_ptr<PointUnprojectorBase> _unprojector;
    size_t _seq = 0;

    // Point clouds
    std::vector<std::shared_ptr<Point3fVectorCloud>> pointClouds_;  // Subsequiential point clouds

    using ContainerType = std::vector<Eigen::Vector3d>;
    using TreeNodeType = TreeNode3D<ContainerType>;
    using TreeNodeTypePtr = TreeNodeType *;
    std::vector<std::vector<TreeNodeTypePtr>> kdTreeLeafes_;  // Leafes from the subsequential kd-trees - Index corresponds to pose index
    std::vector<std::shared_ptr<TreeNodeType>> kdTrees_;      // Kd-trees created from subsequential point clouds - Index corresponds to pose index | Probably I don't need them
    std::vector<Eigen::Isometry3d> poses_;

    std::vector<std::shared_ptr<Surfel>> surfels_; // Vector of all the surfels. Indexes do NOT correspond to anything else
    std::vector<sensor_msgs::PointCloud2> rosPointClouds_; // Vector of point clouds for vizualization, as Rviz sometimes doesn't display them
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pointCloudPub_; // Raw point clouds publisher
    ros::Publisher synthPointCloudPub_;  // Synthetic point clouds publisher

    ros::Publisher odomPub_; // Odometry from .bag file publisher
    ros::Publisher poseArrayPub_;
    ros::Publisher afterOptimPoseArrayPub_;
    ros::Publisher beforeOptimPoseArrayPub_;

    ros::Time lastTimestamp_;
    tf2_ros::TransformBroadcaster transformBroadcaster_;

    // Rviz Visualization Tools
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  };

  using PointCloudProcPtr = std::shared_ptr<PointCloudProc>;

} // namespace structure_refinement
