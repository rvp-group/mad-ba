#pragma once
#include "point_types.h"
#include "point_unprojector.h"
#include <memory>

namespace srrg2_core {

  using Point2fUnprojectorPinhole =
    PointUnprojectorPinhole_<Point2fVectorCloud>;
  using PointNormal2fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormal2fVectorCloud>;
  using PointIntensity2fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointIntensity2fVectorCloud>;
  using PointColor2fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointColor2fVectorCloud>;
  using PointNormalIntensity2fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormalIntensity2fVectorCloud>;
  using PointNormalColor2fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormalColor2fVectorCloud>;

  using Point2fUnprojectorPinholePtr =
    std::shared_ptr<Point2fUnprojectorPinhole>;

  using Point2fUnprojectorPolar = PointUnprojectorPolar_<Point2fVectorCloud>;
  using PointNormal2fUnprojectorPolar =
    PointUnprojectorPolar_<PointNormal2fVectorCloud>;
  using PointIntensity2fUnprojectorPolar =
    PointUnprojectorPolar_<PointIntensity2fVectorCloud>;
  using PointColor2fUnprojectorPolar =
    PointUnprojectorPolar_<PointColor2fVectorCloud>;
  using PointNormalIntensity2fUnprojectorPolar =
    PointUnprojectorPolar_<PointNormalIntensity2fVectorCloud>;
  using PointNormalColor2fUnprojectorPolar =
    PointUnprojectorPolar_<PointNormalColor2fVectorCloud>;

  using Point2fUnprojectorPolarPtr = std::shared_ptr<Point2fUnprojectorPolar>;
  using PointNormal2fUnprojectorPolarPtr =
    std::shared_ptr<PointNormal2fUnprojectorPolar>;

  using Point3fUnprojectorPinhole =
    PointUnprojectorPinhole_<Point3fVectorCloud>;
  using PointNormal3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormal3fVectorCloud>;
  using PointIntensity3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointIntensity3fVectorCloud>;
  using PointColor3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointColor3fVectorCloud>;
  using PointNormalIntensity3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointIntensityDescriptor3fVectorCloud>;

  using PointIntensityDescriptor3fUnprojectorPinholePtr =
    std::shared_ptr<PointIntensityDescriptor3fUnprojectorPinhole>;

  using PointNormalCurvatureColor3fUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dUnprojectorPinhole =
    PointUnprojectorPinhole_<PointNormalCurvatureColor3dVectorCloud>;
  using PointNormalCurvatureColor3fUnprojectorPinholePtr =
    std::shared_ptr<PointNormalCurvatureColor3fUnprojectorPinhole>;
  using PointNormalCurvatureColor3dUnprojectorPinholePtr =
    std::shared_ptr<PointNormalCurvatureColor3dUnprojectorPinhole>;

} // namespace srrg2_core
