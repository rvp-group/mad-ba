#include "instances.h"
#include "srrg_boss/blob.h"

namespace srrg2_core {

  void point_cloud_registerTypes() {
    //! @brief registering blob references
    BOSS_REGISTER_BLOB(Point2fVectorData);
    BOSS_REGISTER_BLOB(PointColor2fVectorData);
    BOSS_REGISTER_BLOB(PointNormal2fVectorData);
    BOSS_REGISTER_BLOB(PointNormalColor2fVectorData);
    BOSS_REGISTER_BLOB(PointNormalCurvature2fVectorData);

    BOSS_REGISTER_BLOB(Point2dVectorData);
    BOSS_REGISTER_BLOB(PointColor2dVectorData);
    BOSS_REGISTER_BLOB(PointNormal2dVectorData);
    BOSS_REGISTER_BLOB(PointNormalColor2dVectorData);
    BOSS_REGISTER_BLOB(PointNormalCurvature2dVectorData);
    BOSS_REGISTER_BLOB(PointId2fVectorData);

    BOSS_REGISTER_BLOB(Point3dVectorData);
    BOSS_REGISTER_BLOB(PointColor3dVectorData);
    BOSS_REGISTER_BLOB(PointNormal3dVectorData);
    BOSS_REGISTER_BLOB(PointNormalColor3dVectorData);
    BOSS_REGISTER_BLOB(PointNormalCurvature3dVectorData);
    BOSS_REGISTER_BLOB(PointNormalCurvatureColor3dVectorData);

    BOSS_REGISTER_BLOB(Point3fVectorData);
    BOSS_REGISTER_BLOB(PointColor3fVectorData);
    BOSS_REGISTER_BLOB(PointNormal3fVectorData);
    BOSS_REGISTER_BLOB(PointNormalColor3fVectorData);
    BOSS_REGISTER_BLOB(PointNormalCurvature3fVectorData);
    BOSS_REGISTER_BLOB(PointIntensityDescriptor3fVectorData);
    BOSS_REGISTER_BLOB(PointNormalCurvatureColor3fVectorData);
    BOSS_REGISTER_BLOB(PointId3fVectorData);

    //! @brief registering classes
    BOSS_REGISTER_CLASS(Point2fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormal2fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointIntensity2fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointColor2fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalIntensity2fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalColor2fProjectorPinhole);

    BOSS_REGISTER_CLASS(Point2fProjectorPolar);
    BOSS_REGISTER_CLASS(PointNormal2fProjectorPolar);
    BOSS_REGISTER_CLASS(PointIntensity2fProjectorPolar);
    BOSS_REGISTER_CLASS(PointColor2fProjectorPolar);
    BOSS_REGISTER_CLASS(PointNormalIntensity2fProjectorPolar);
    BOSS_REGISTER_CLASS(PointNormalColor2fProjectorPolar);

    BOSS_REGISTER_CLASS(Point3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormal3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointIntensity3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointColor3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalIntensity3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalColor3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointIntensityDescriptor3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fProjectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3dProjectorPinhole);

    BOSS_REGISTER_CLASS(Point2fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormal2fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointIntensity2fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointColor2fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalIntensity2fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalColor2fUnprojectorPinhole);

    BOSS_REGISTER_CLASS(Point2fUnprojectorPolar);
    BOSS_REGISTER_CLASS(PointNormal2fUnprojectorPolar);
    BOSS_REGISTER_CLASS(PointIntensity2fUnprojectorPolar);
    BOSS_REGISTER_CLASS(PointColor2fUnprojectorPolar);
    BOSS_REGISTER_CLASS(PointNormalIntensity2fUnprojectorPolar);
    BOSS_REGISTER_CLASS(PointNormalColor2fUnprojectorPolar);

    BOSS_REGISTER_CLASS(Point3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormal3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointIntensity3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointColor3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalIntensity3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalColor3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointIntensityDescriptor3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fUnprojectorPinhole);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3dUnprojectorPinhole);

    BOSS_REGISTER_CLASS(CameraMatrixOwner3D);

    BOSS_REGISTER_CLASS(NormalComputator1DSlidingWindowNormal);

    // ia lidar projectors ouster
    BOSS_REGISTER_CLASS(Point3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormal3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointIntensity3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointColor3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalIntensity3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalColor3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointIntensityDescriptor3fProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fProjectorOS1_16);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fProjectorOS1_64);

    // ia lidar projector kitti
    BOSS_REGISTER_CLASS(PointIntensity3fProjectorHDL64E);

    // ia misc projectors
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3dProjectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fProjectorVLP16);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fProjectorHDL32);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fProjectorHDL64E);

    // ia lidar unprojectors
    BOSS_REGISTER_CLASS(Point3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormal3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointIntensity3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointColor3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalIntensity3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalColor3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointIntensityDescriptor3fUnprojectorOS1_64);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fUnprojectorOS1_64);

    // ia lidar unprojector kitti
    BOSS_REGISTER_CLASS(PointIntensity3fUnprojectorHDL64E);

    // ia misc unprojectors
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3dUnprojectorOS1_64);

    // ia shaslam auxiliary projectors
    BOSS_REGISTER_CLASS(Point3fProjectorOS1_16);
    BOSS_REGISTER_CLASS(Point3fProjectorVLP16);
    BOSS_REGISTER_CLASS(Point3fProjectorHDL32);
    BOSS_REGISTER_CLASS(Point3fProjectorHDL64E);

    // ia shaslam auxiliary unprojectors
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fUnprojectorOS1_16);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fUnprojectorVLP16);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fUnprojectorHDL32);
    BOSS_REGISTER_CLASS(PointNormalCurvatureColor3fUnprojectorHDL64E);


  }
} // namespace srrg2_core
