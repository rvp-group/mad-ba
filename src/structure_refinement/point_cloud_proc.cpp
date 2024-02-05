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

namespace structure_refinement {
  using namespace srrg2_core;

  using LidarProjectorType   = PointIntensity3fProjectorOS1_64;
  using LidarUnprojectorType = PointIntensity3fUnprojectorOS1_64;
  using NormalComputatorType = NormalComputator2DCrossProduct<PointNormal3fVectorCloud,0>; //NormalComputator2DCrossProduct<PointNormal3fMatrixCloud, 1>;

  PointCloudProc::PointCloudProc() {
  }

  PointCloudProc::~PointCloudProc() {
  }

  

  bool PointCloudProc::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    PointCloud2MessagePtr cloud = std::dynamic_pointer_cast<PointCloud2Message>(msg_);
    if (!cloud) {
      std::cerr << "PointCloudProc::putMessage | no msg received" << std::endl;
      return false;
    }
    std::cerr << "Cloud height: " << cloud->height.value() << " Cloud width: " << cloud->width.value() << std::endl;
    Point3fVectorCloud cloud2;
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
    cv::imshow("lidar_intensity", cv_intensity_image);
    // cv::waitKey(1);

    return true;
  }

//   void getLidarData(LidarProjectorType& projector_,
//                const PointCloud2MessagePtr& lidar_msg_,
//                const std::string& directory_,
//                const bool dump_depth_) {
//   LidarProjectorType::Lidar3DSensorType lidar_sensor;
//   // ia projected point cloud (within a special data structure containing auxliary fields also)
//   LidarProjectorType::TargetMatrixType projection_target_matrix;
//   // ia intensity and depth image from the projected cloud
//   srrg2_core::ImageFloat intensity_image;
//   srrg2_core::ImageFloat depth_image;

//   // ia extract a usable pointcloud from the raw message
//   PointIntensity3fVectorCloud current_cloud;
//   lidar_msg_->getPointCloud(current_cloud);

//   float max_intensity = -1.f;
//   for (const auto& p : current_cloud) {
//     if (p.intensity() > max_intensity) {
//       max_intensity = p.intensity();
//     }
//   }

//   // ia project the cloud
//   projector_.compute(projection_target_matrix, current_cloud.begin(), current_cloud.end());

//   // ia get the intensity image
//   intensity_image.resize(lidar_sensor.verticalResolution(), projector_.param_num_columns.value());
//   auto intensity_image_it = intensity_image.begin();
//   for (auto it = projection_target_matrix.begin(); it != projection_target_matrix.end();
//        ++it, ++intensity_image_it) {
//     *intensity_image_it = it->source_it->intensity();
//   }

//   // ia get the depth image
//   projection_target_matrix.toDepthMatrix(depth_image);

//   // ia use opencv to show something
//   intensity_image.toCv(cv_intensity_image);
//   depth_image.toCv(cv_depth_image);

//   // ia save the intensity image in the output directory
//   cv::imwrite(directory_ + "/intensity_" + std::to_string(msg_number) + ".png", cv_intensity_image);
//   if (dump_depth_) {
//     cv::imwrite(directory_ + "/depth_" + std::to_string(msg_number) + ".png", cv_depth_image);
//   }

//   cv_depth_image     = cv_depth_image / projector_.param_range_max.value();
//   cv_intensity_image = cv_intensity_image / max_intensity;
// }

} // namespace structure_refinement
