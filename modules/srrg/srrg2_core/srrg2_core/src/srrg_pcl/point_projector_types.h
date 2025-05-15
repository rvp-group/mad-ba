#pragma once
#include "point_projector_pinhole.h"
#include "point_projector_polar.h"
#include "point_types.h"

namespace srrg2_core {

  using Point2fProjectorPinhole = PointProjectorPinhole_<Point2fVectorCloud>;
  using PointNormal2fProjectorPinhole =
    PointProjectorPinhole_<PointNormal2fVectorCloud>;
  using PointIntensity2fProjectorPinhole =
    PointProjectorPinhole_<PointIntensity2fVectorCloud>;
  using PointColor2fProjectorPinhole =
    PointProjectorPinhole_<PointColor2fVectorCloud>;
  using PointNormalIntensity2fProjectorPinhole =
    PointProjectorPinhole_<PointNormalIntensity2fVectorCloud>;
  using PointNormalColor2fProjectorPinhole =
    PointProjectorPinhole_<PointNormalColor2fVectorCloud>;

  using Point2fProjectorPinholePtr = std::shared_ptr<Point2fProjectorPinhole>;

  using Point2fProjectorPolar = PointProjectorPolar_<Point2fVectorCloud>;
  using PointNormal2fProjectorPolar =
    PointProjectorPolar_<PointNormal2fVectorCloud>;
  using PointIntensity2fProjectorPolar =
    PointProjectorPolar_<PointIntensity2fVectorCloud>;
  using PointColor2fProjectorPolar =
    PointProjectorPolar_<PointColor2fVectorCloud>;
  using PointNormalIntensity2fProjectorPolar =
    PointProjectorPolar_<PointNormalIntensity2fVectorCloud>;
  using PointNormalColor2fProjectorPolar =
    PointProjectorPolar_<PointNormalColor2fVectorCloud>;

  using Point2fProjectorPolarPtr = std::shared_ptr<Point2fProjectorPolar>;
  using PointNormal2fProjectorPolarPtr =
    std::shared_ptr<PointNormal2fProjectorPolar>;

  using Point3fProjectorPinhole = PointProjectorPinhole_<Point3fVectorCloud>;
  using PointNormal3fProjectorPinhole =
    PointProjectorPinhole_<PointNormal3fVectorCloud>;
  using PointIntensity3fProjectorPinhole =
    PointProjectorPinhole_<PointIntensity3fVectorCloud>;
  using PointColor3fProjectorPinhole =
    PointProjectorPinhole_<PointColor3fVectorCloud>;
  using PointNormalIntensity3fProjectorPinhole =
    PointProjectorPinhole_<PointNormalIntensity3fVectorCloud>;
  using PointNormalColor3fProjectorPinhole =
    PointProjectorPinhole_<PointNormalColor3fVectorCloud>;
  using PointIntensityDescriptor3fProjectorPinhole =
    PointProjectorPinhole_<PointIntensityDescriptor3fVectorCloud>;

  using PointIntensityDescriptor3fProjectorPinholePtr =
    std::shared_ptr<PointIntensityDescriptor3fProjectorPinhole>;

  using PointNormalCurvatureColor3fProjectorPinhole =
    PointProjectorPinhole_<PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3dProjectorPinhole =
    PointProjectorPinhole_<PointNormalCurvatureColor3dVectorCloud>;
  using PointNormalCurvatureColor3fProjectorPinholePtr =
    std::shared_ptr<PointNormalCurvatureColor3fProjectorPinhole>;
  using PointNormalCurvatureColor3dProjectorPinholePtr =
    std::shared_ptr<PointNormalCurvatureColor3dProjectorPinhole>;

} // namespace srrg2_core
