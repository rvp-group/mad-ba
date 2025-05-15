#pragma once
#include <memory>

#include "point_types.h"
#include "point_unprojector_lidar3d.h"

namespace srrg2_core {

  // Velodyne VLP-16
  using Point3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, Point3fVectorCloud>;
  using PointNormal3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointNormal3fVectorCloud>;
  using PointIntensity3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointIntensity3fVectorCloud>;
  using PointColor3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16, PointColor3fVectorCloud>;
  using PointNormalIntensity3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                             PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                             PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                             PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                             PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dUnprojectorVLP16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::VLP_16,
                             PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptr
  using PointIntensityDescriptor3fUnprojectorVLP16Ptr =
    std::shared_ptr<PointIntensityDescriptor3fUnprojectorVLP16>;
  using PointNormalCurvatureColor3fUnprojectorVLP16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fUnprojectorVLP16>;
  using PointNormalCurvatureColor3dUnprojectorVLP16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dUnprojectorVLP16>;

  // Velodyne HDL-32
  using Point3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, Point3fVectorCloud>;
  using PointNormal3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointNormal3fVectorCloud>;
  using PointIntensity3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointIntensity3fVectorCloud>;
  using PointColor3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32, PointColor3fVectorCloud>;
  using PointNormalIntensity3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                             PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                             PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                             PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                             PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dUnprojectorHDL32 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_32,
                             PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptr
  using PointIntensityDescriptor3fUnprojectorHDL32Ptr =
    std::shared_ptr<PointIntensityDescriptor3fUnprojectorHDL32>;
  using PointNormalCurvatureColor3fUnprojectorHDL32Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fUnprojectorHDL32>;
  using PointNormalCurvatureColor3dUnprojectorHDL32Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dUnprojectorHDL32>;

  // Velodyne HDL-64-E
  using Point3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, Point3fVectorCloud>;
  using PointNormal3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointNormal3fVectorCloud>;
  using PointIntensity3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointIntensity3fVectorCloud>;
  using PointColor3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E, PointColor3fVectorCloud>;
  using PointNormalIntensity3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                             PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                             PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                             PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                             PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dUnprojectorHDL64E =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::HDL_64E,
                             PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptr
  using PointIntensityDescriptor3fUnprojectorHDL64EPtr =
    std::shared_ptr<PointIntensityDescriptor3fUnprojectorHDL64E>;
  using PointNormalCurvatureColor3fUnprojectorHDL64EPtr =
    std::shared_ptr<PointNormalCurvatureColor3fUnprojectorHDL64E>;
  using PointNormalCurvatureColor3dUnprojectorHDL64EPtr =
    std::shared_ptr<PointNormalCurvatureColor3dUnprojectorHDL64E>;

  // Ouster OS1-16
  using Point3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, Point3fVectorCloud>;
  using PointNormal3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointNormal3fVectorCloud>;
  using PointIntensity3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointIntensity3fVectorCloud>;
  using PointColor3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16, PointColor3fVectorCloud>;
  using PointNormalIntensity3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                             PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                             PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                             PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                             PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dUnprojectorOS1_16 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_16,
                             PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptr
  using PointIntensityDescriptor3fUnprojectorOS1_16Ptr =
    std::shared_ptr<PointIntensityDescriptor3fUnprojectorOS1_16>;
  using PointNormalCurvatureColor3fUnprojectorOS1_16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fUnprojectorOS1_16>;
  using PointNormalCurvatureColor3dUnprojectorOS1_16Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dUnprojectorOS1_16>;

  // Ouster OS1-64
  using Point3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, Point3fVectorCloud>;
  using PointNormal3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointNormal3fVectorCloud>;
  using PointIntensity3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointIntensity3fVectorCloud>;
  using PointColor3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64, PointColor3fVectorCloud>;
  using PointNormalIntensity3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                             PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                             PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                             PointIntensityDescriptor3fVectorCloud>;
  using PointNormalCurvatureColor3fUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                             PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dUnprojectorOS1_64 =
    PointUnprojectorLidar3D_<srrg2_lidar3d_utils::LIDAR_TYPE::OS1_64,
                             PointNormalCurvatureColor3dVectorCloud>;

  // ia shared ptr
  using PointIntensityDescriptor3fUnprojectorOS1_64Ptr =
    std::shared_ptr<PointIntensityDescriptor3fUnprojectorOS1_64>;
  using PointNormalCurvatureColor3fUnprojectorOS1_64Ptr =
    std::shared_ptr<PointNormalCurvatureColor3fUnprojectorOS1_64>;
  using PointNormalCurvatureColor3dUnprojectorOS1_64Ptr =
    std::shared_ptr<PointNormalCurvatureColor3dUnprojectorOS1_64>;
} // namespace srrg2_core
