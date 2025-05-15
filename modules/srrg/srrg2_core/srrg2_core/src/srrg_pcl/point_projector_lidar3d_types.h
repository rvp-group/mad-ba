#pragma once
#include <memory>

#include "point_types.h"

#include "point_projector_lidar3d.h"

namespace srrg2_core {

  // Velodyne VLP-16
  using Point3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, Point3fVectorCloud>;
  using PointNormal3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointNormal3fVectorCloud>;
  using PointIntensity3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointIntensity3fVectorCloud>;
  using PointColor3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointColor3fVectorCloud>;
  using PointNormalIntensity3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                           PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                           PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                           PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dProjectorVLP16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                           PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptrs
  using PointIntensityDescriptor3fProjectorVLP16Ptr =
    std::shared_ptr<PointIntensityDescriptor3fProjectorVLP16>;
  using PointNormalCurvatureColor3fProjectorVLP16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fProjectorVLP16>;
  using PointNormalCurvatureColor3dProjectorVLP16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dProjectorVLP16>;
  using PointIntensity3fProjectorVLP16Ptr = std::shared_ptr<PointIntensity3fProjectorVLP16>;

  // Velodyne HDL-32
  using Point3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, Point3fVectorCloud>;
  using PointNormal3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointNormal3fVectorCloud>;
  using PointIntensity3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointIntensity3fVectorCloud>;
  using PointColor3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointColor3fVectorCloud>;
  using PointNormalIntensity3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                           PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                           PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                           PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dProjectorHDL32 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                           PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptrs
  using PointIntensityDescriptor3fProjectorHDL32Ptr =
    std::shared_ptr<PointIntensityDescriptor3fProjectorHDL32>;
  using PointNormalCurvatureColor3fProjectorHDL32Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fProjectorHDL32>;
  using PointNormalCurvatureColor3dProjectorHDL32Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dProjectorHDL32>;
  using PointIntensity3fProjectorHDL32Ptr = std::shared_ptr<PointIntensity3fProjectorHDL32>;

  // Velodyne HDL-64-E
  using Point3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, Point3fVectorCloud>;
  using PointNormal3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointNormal3fVectorCloud>;
  using PointIntensity3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointIntensity3fVectorCloud>;
  using PointColor3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointColor3fVectorCloud>;
  using PointNormalIntensity3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                           PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                           PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                           PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dProjectorHDL64E =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                           PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptrs
  using PointIntensityDescriptor3fProjectorHDL64EPtr =
    std::shared_ptr<PointIntensityDescriptor3fProjectorHDL64E>;
  using PointNormalCurvatureColor3fProjectorHDL64EPtr =
    std::shared_ptr<PointNormalCurvatureColor3fProjectorHDL64E>;
  using PointNormalCurvatureColor3dProjectorHDL64EPtr =
    std::shared_ptr<PointNormalCurvatureColor3dProjectorHDL64E>;
  using PointIntensity3fProjectorHDL64EPtr = std::shared_ptr<PointIntensity3fProjectorHDL64E>;

  // Ouster OS1-16
  using Point3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, Point3fVectorCloud>;
  using PointNormal3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointNormal3fVectorCloud>;
  using PointIntensity3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointIntensity3fVectorCloud>;
  using PointColor3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointColor3fVectorCloud>;
  using PointNormalIntensity3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                           PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                           PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                           PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dProjectorOS1_16 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                           PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptrs
  using PointIntensityDescriptor3fProjectorOS1_16Ptr =
    std::shared_ptr<PointIntensityDescriptor3fProjectorOS1_16>;
  using PointNormalCurvatureColor3fProjectorOS1_16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fProjectorOS1_16>;
  using PointNormalCurvatureColor3dProjectorOS1_16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dProjectorOS1_16>;
  using PointIntensity3fProjectorOS1_16Ptr = std::shared_ptr<PointIntensity3fProjectorOS1_16>;

  // Ouster OS1-64
  using Point3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, Point3fVectorCloud>;
  using PointNormal3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointNormal3fVectorCloud>;
  using PointIntensity3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointIntensity3fVectorCloud>;
  using PointColor3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointColor3fVectorCloud>;
  using PointNormalIntensity3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                           PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                           PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                           PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dProjectorOS1_64 =
    PointProjectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                           PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptrs
  using PointIntensityDescriptor3fProjectorOS1_64Ptr =
    std::shared_ptr<PointIntensityDescriptor3fProjectorOS1_64>;
  using PointNormalCurvatureColor3fProjectorOS1_64Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fProjectorOS1_64>;
  using PointNormalCurvatureColor3dProjectorOS1_64Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dProjectorOS1_64>;
  using PointIntensity3fProjectorOS1_64Ptr = std::shared_ptr<PointIntensity3fProjectorOS1_64>;

  /**/

} // namespace srrg2_core
