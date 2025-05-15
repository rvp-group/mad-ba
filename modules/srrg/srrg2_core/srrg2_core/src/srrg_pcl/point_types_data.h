#include "point_types.h"
#include "srrg_property/vector_data.h"

namespace srrg2_core {

  //! @brief vector data usings
  using Point2fVectorData       = VectorData_<Point2fVectorCloud>;
  using PointId2fVectorData       = VectorData_<PointId2fVectorCloud>;
  using PointColor2fVectorData  = VectorData_<PointColor2fVectorCloud>;
  using PointNormal2fVectorData = VectorData_<PointNormal2fVectorCloud>;
  using PointNormalColor2fVectorData =
    VectorData_<PointNormalColor2fVectorCloud>;
  using PointNormalCurvature2fVectorData =
    VectorData_<PointNormalCurvature2fVectorCloud>;

  using Point2dVectorData       = VectorData_<Point2dVectorCloud>;
  using PointColor2dVectorData  = VectorData_<PointColor2dVectorCloud>;
  using PointNormal2dVectorData = VectorData_<PointNormal2dVectorCloud>;
  using PointNormalColor2dVectorData =
    VectorData_<PointNormalColor2dVectorCloud>;
  using PointNormalCurvature2dVectorData =
    VectorData_<PointNormalCurvature2dVectorCloud>;

  using Point3fVectorData          = VectorData_<Point3fVectorCloud>;
  using PointId3fVectorData       = VectorData_<PointId3fVectorCloud>;
  using PointColor3fVectorData     = VectorData_<PointColor3fVectorCloud>;
  using PointIntensity3fVectorData = VectorData_<PointIntensity3fVectorCloud>;
  using PointNormal3fVectorData    = VectorData_<PointNormal3fVectorCloud>;
  using PointNormalColor3fVectorData =
    VectorData_<PointNormalColor3fVectorCloud>;
  using PointNormalIntensity3fVectorData =
    VectorData_<PointNormalIntensity3fVectorCloud>;
  using PointNormalCurvature3fVectorData =
    VectorData_<PointNormalCurvature3fVectorCloud>;
  using PointIntensityDescriptor3fVectorData =
    VectorData_<PointIntensityDescriptor3fVectorCloud>;

  using Point3dVectorData          = VectorData_<Point3dVectorCloud>;
  using PointColor3dVectorData     = VectorData_<PointColor3dVectorCloud>;
  using PointIntensity3dVectorData = VectorData_<PointIntensity3dVectorCloud>;
  using PointNormal3dVectorData    = VectorData_<PointNormal3dVectorCloud>;
  using PointNormalColor3dVectorData =
    VectorData_<PointNormalColor3dVectorCloud>;
  using PointNormalIntensity3dVectorData =
    VectorData_<PointNormalIntensity3dVectorCloud>;
  using PointNormalCurvature3dVectorData =
    VectorData_<PointNormalCurvature3dVectorCloud>;

  //! @brief boss blob reference
  using Point2dVectorBLOBReference      = BLOBReference<Point2dVectorData>;
  using PointColor2dVectorBLOBReference = BLOBReference<PointColor2dVectorData>;
  using PointNormal2dVectorBLOBReference =
    BLOBReference<PointNormal2dVectorData>;
  using PointNormalColor2dVectorBLOBReference =
    BLOBReference<PointNormalColor2dVectorData>;
  using PointNormalCurvature2dVectorBLOBReference =
    BLOBReference<PointNormalCurvature2dVectorData>;

  using Point2fVectorBLOBReference      = BLOBReference<Point2fVectorData>;
  using PointId2fVectorBLOBReference      = BLOBReference<PointId2fVectorData>;
  using PointColor2fVectorBLOBReference = BLOBReference<PointColor2fVectorData>;
  using PointNormal2fVectorBLOBReference =
    BLOBReference<PointNormal2fVectorData>;
  using PointNormalColor2fVectorBLOBReference =
    BLOBReference<PointNormalColor2fVectorData>;
  using PointNormalCurvature2fVectorBLOBReference =
    BLOBReference<PointNormalCurvature2fVectorData>;

  using Point3fVectorBLOBReference      = BLOBReference<Point3fVectorData>;
  using PointId3fVectorBLOBReference      = BLOBReference<PointId3fVectorData>;
  using PointColor3fVectorBLOBReference = BLOBReference<PointColor3fVectorData>;
  using PointIntensity3fVectorBLOBReference =
    BLOBReference<PointIntensity3fVectorData>;
  using PointNormal3fVectorBLOBReference =
    BLOBReference<PointNormal3fVectorData>;
  using PointNormalColor3fVectorBLOBReference =
    BLOBReference<PointNormalColor3fVectorData>;
  using PointNormalIntensity3fVectorBLOBReference =
    BLOBReference<PointNormalIntensity3fVectorData>;
  using PointNormalCurvature3fVectorBLOBReference =
    BLOBReference<PointNormalCurvature3fVectorData>;
  using PointIntensityDescriptor3fVectorBLOBReference =
    BLOBReference<PointIntensityDescriptor3fVectorData>;

  using Point3dVectorBLOBReference      = BLOBReference<Point3dVectorData>;
  using PointColor3dVectorBLOBReference = BLOBReference<PointColor3dVectorData>;
  using PointIntensity3dVectorBLOBReference =
    BLOBReference<PointIntensity3dVectorData>;
  using PointNormal3dVectorBLOBReference =
    BLOBReference<PointNormal3dVectorData>;
  using PointNormalColor3dVectorBLOBReference =
    BLOBReference<PointNormalColor3dVectorData>;
  using PointNormalIntensity3dVectorBLOBReference =
    BLOBReference<PointNormalIntensity3dVectorData>;
  using PointNormalCurvature3dVectorBLOBReference =
    BLOBReference<PointNormalCurvature3dVectorData>;

  // ia serialization of shaslam points
  using PointNormalCurvatureColor3fVectorData =
    VectorData_<PointNormalCurvatureColor3fVectorCloud>;
  using PointNormalCurvatureColor3fVectorBLOBReference =
    BLOBReference<PointNormalCurvatureColor3fVectorData>;
  using PointNormalCurvatureColor3dVectorData =
    VectorData_<PointNormalCurvatureColor3dVectorCloud>;
  using PointNormalCurvatureColor3dVectorBLOBReference =
    BLOBReference<PointNormalCurvatureColor3dVectorData>;

  
} // namespace srrg2_core
