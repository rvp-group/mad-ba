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

  bool PointCloudProc::putMessage(srrg2_core::BaseSensorMessagePtr msg) {


      // Convert message to srrg2 point cloud
      PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg);
      if (!cloud) {
          std::cerr << "PointCloudProc::putMessage | no msg received" << std::endl;
          return false;
      }
      
      // Print the general info for debugging
      static int cnt = 0;
      std::cout << std::setprecision(12) << "Cnt: " << ++cnt << " TS: " << cloud->timestamp.value()
                << " Cloud height: " << cloud->height.value() << " Cloud width: " << cloud->width.value() << std::endl;


      // Change the frame_id of the point cloud
      cloud->frame_id.value() = "map";
      // Convert srrg2 to sensor_msgs point cloud
      sensor_msgs::PointCloud2ConstPtr ros_msg = Converter::convert(cloud);

      // Publish the ROS message
      pointCloudPub_.publish(*ros_msg);
      publishNormals();

      return true;
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
