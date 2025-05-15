#include "srrg_pcl/instances.h"
#include "srrg_test/synthetic_world.hpp"

using namespace srrg2_core;
using namespace srrg2_test;

int main(int argc_, char** argv_) {
  // ds enable profiler to measure actual projection times
  Profiler::enable_logging = true;
  return srrg2_test::runTests(argc_, argv_);
}

TEST_F(SyntheticWorldSE2, PointProjectorPolarDepthBuffer_100) {
  // ds test configuration
  constexpr size_t num_beams        = 100;
  constexpr float max_intensity     = 10; // meters
  constexpr float sensor_resolution = 2 * M_PI / num_beams;
  projection_matrix << 1.f / sensor_resolution, num_beams / 2.f, 0.f, 0.f;

  // ds generate a random world scenario
  const Vector2f mean(max_intensity - 5, max_intensity - 5);
  const Vector2f deviation(5, 5);
  generateMap(num_beams, mean, deviation, SensorType::Laser);

  // ds all points are valid
  ASSERT_EQ(points_in_sensor[0].size(), num_beams);

  // ds initialize projector
  PointProjectorPolar_<Point2fVectorCloud> projector_2d;
  projector_2d.param_range_max.setValue(maximum_projection_value[0] + 1);
  projector_2d.setCanvasCols(num_beams);
  projector_2d.setCanvasRows(1);
  projector_2d.setCameraPose(sensor_poses[0]);
  projector_2d.initCameraMatrix();

  // ds perform projection
  PointProjectorPolar_<Point2fVectorCloud>::TargetMatrixType projected_ranges;
  const size_t number_of_valid_projections =
    projector_2d.compute(projected_ranges, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections (depth buffers cancels some)
  ASSERT_EQ(number_of_valid_projections, static_cast<size_t>(24));
}

TEST_F(SyntheticWorldSE2, PointProjectorPolarDepthBuffer_1000) {
  // ds test configuration
  constexpr size_t num_beams        = 1000;
  constexpr float max_intensity     = 10; // meters
  constexpr float sensor_resolution = 2 * M_PI / num_beams;
  projection_matrix << 1.f / sensor_resolution, num_beams / 2.f, 0.f, 0.f;

  // ds generate a random world scenario
  const Vector2f mean(max_intensity - 5, max_intensity - 5);
  const Vector2f deviation(5, 5);
  generateMap(num_beams, mean, deviation, SensorType::Laser);

  // ds all points are valid
  ASSERT_EQ(points_in_sensor[0].size(), num_beams);

  // ds initialize projector
  PointProjectorPolar_<Point2fVectorCloud> projector_2d;
  projector_2d.param_range_max.setValue(maximum_projection_value[0] + 1);
  projector_2d.setCanvasCols(num_beams);
  projector_2d.setCanvasRows(1);
  projector_2d.setCameraPose(sensor_poses[0]);
  projector_2d.initCameraMatrix();

  // ds perform projection
  PointProjectorPolar_<Point2fVectorCloud>::TargetMatrixType projected_ranges;
  const size_t number_of_valid_projections =
    projector_2d.compute(projected_ranges, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_LE(number_of_valid_projections, static_cast<size_t>(10000));
}

TEST_F(SyntheticWorldSE2, PointProjectorPolarDepthBuffer_10000) {
  // ds test configuration
  constexpr size_t num_beams        = 10000;
  constexpr float max_intensity     = 10; // meters
  constexpr float sensor_resolution = 2 * M_PI / num_beams;
  projection_matrix << 1.f / sensor_resolution, num_beams / 2.f, 0.f, 0.f;

  // ds generate a random world scenario
  const Vector2f mean(max_intensity - 5, max_intensity - 5);
  const Vector2f deviation(5, 5);
  generateMap(num_beams, mean, deviation, SensorType::Laser);

  // ds all points are valid
  ASSERT_EQ(points_in_sensor[0].size(), num_beams);

  // ds initialize projector
  PointProjectorPolar_<Point2fVectorCloud> projector_2d;
  projector_2d.param_range_max.setValue(maximum_projection_value[0] + 1);
  projector_2d.setCanvasCols(num_beams);
  projector_2d.setCanvasRows(1);
  projector_2d.setCameraPose(sensor_poses[0]);
  projector_2d.initCameraMatrix();

  // ds perform projection
  PointProjectorPolar_<Point2fVectorCloud>::TargetMatrixType projected_ranges;
  const size_t number_of_valid_projections =
    projector_2d.compute(projected_ranges, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_LE(number_of_valid_projections, static_cast<size_t>(10000));
}

using SyntheticWorldSE2Descriptors =
  SyntheticWorld<2, float, PointIntensityDescriptor2f, PointIntensityDescriptor1f>;
TEST_F(SyntheticWorldSE2Descriptors, PointProjectorPolarDepthBuffer_10000) {
  // ds test configuration
  constexpr size_t num_beams        = 10000;
  constexpr float max_intensity     = 10; // meters
  constexpr float sensor_resolution = 2 * M_PI / num_beams;
  projection_matrix << 1.f / sensor_resolution, num_beams / 2.f, 0.f, 0.f;

  // ds generate a random world scenario
  const Vector2f mean(max_intensity - 5, max_intensity - 5);
  const Vector2f deviation(5, 5);
  generateMap(num_beams, mean, deviation, SensorType::Laser);

  // ds all points are valid
  ASSERT_EQ(points_in_sensor[0].size(), num_beams);

  // ds initialize projector
  PointProjectorPolar_<PointIntensityDescriptor2fVectorCloud> projector_2d;
  projector_2d.param_range_max.setValue(maximum_projection_value[0] + 1);
  projector_2d.setCanvasCols(num_beams);
  projector_2d.setCanvasRows(1);
  projector_2d.setCameraPose(sensor_poses[0]);
  projector_2d.initCameraMatrix();

  // ds perform projection
  PointProjectorPolar_<PointIntensityDescriptor2fVectorCloud>::TargetMatrixType projected_ranges;
  const size_t number_of_valid_projections =
    projector_2d.compute(projected_ranges, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_LE(number_of_valid_projections, static_cast<size_t>(10000));
}

TEST_F(SyntheticWorldSE3, PointProjectorPinholeDepthBuffer_100) {
  // ds test configuration
  constexpr size_t number_of_points = 100;
  constexpr float distance          = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(99));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(99));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  Point3fProjectorPinhole::TargetMatrixType projected_points;
  const size_t number_of_valid_projections =
    projector_3d.compute(projected_points, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinholeDepthBuffer_1000) {
  // ds test configuration
  constexpr size_t number_of_points = 1000;
  constexpr float distance          = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(494));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(494));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  Point3fProjectorPinhole::TargetMatrixType projected_points;
  const size_t number_of_valid_projections =
    projector_3d.compute(projected_points, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinholeDepthBuffer_10000) {
  // ds test configuration
  constexpr size_t number_of_points = 10000;
  constexpr float distance          = 100;
  projection_matrix << 500, 0, 250, 0, 500, 250, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(1771));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(791));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  Point3fProjectorPinhole::TargetMatrixType projected_points;
  const size_t number_of_valid_projections =
    projector_3d.compute(projected_points, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

using SyntheticWorldSE3Descriptors =
  SyntheticWorld<3, float, PointIntensityDescriptor3f, PointIntensityDescriptor2f>;
TEST_F(SyntheticWorldSE3Descriptors, PointProjectorPinholeDepthBuffer_10000) {
  // ds test configuration
  constexpr size_t number_of_points = 10000;
  constexpr float distance          = 100;
  projection_matrix << 500, 0, 250, 0, 500, 250, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(1771));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(791));

  // ds initialize projector
  PointIntensityDescriptor3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  PointIntensityDescriptor3fProjectorPinhole::TargetMatrixType projected_points;
  const size_t number_of_valid_projections =
    projector_3d.compute(projected_points, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinhole_100) {
  // ds test configuration
  constexpr size_t number_of_points = 100;
  constexpr float distance          = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(99));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(99));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  Point3fVectorCloud points_in_camera;
  Point3fVectorCloud points_in_image;
  std::vector<int> indices;
  const size_t number_of_valid_projections =
    projector_3d.compute(points_in_sensor[0], points_in_camera, points_in_image, indices);

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinhole_1000) {
  // ds test configuration
  constexpr size_t number_of_points = 1000;
  constexpr float distance          = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(494));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(494));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  Point3fVectorCloud points_in_camera;
  Point3fVectorCloud points_in_image;
  std::vector<int> indices;
  const size_t number_of_valid_projections =
    projector_3d.compute(points_in_sensor[0], points_in_camera, points_in_image, indices);

  // ds check number of valid projections (depth buffers cancels some)
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinhole_10000) {
  // ds test configuration
  constexpr size_t number_of_points = 10000;
  constexpr float distance          = 100;
  projection_matrix << 500, 0, 250, 0, 500, 250, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(1771));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(791));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  Point3fVectorCloud points_in_camera;
  Point3fVectorCloud points_in_image;
  std::vector<int> indices;
  const size_t number_of_valid_projections =
    projector_3d.compute(points_in_sensor[0], points_in_camera, points_in_image, indices);

  // ds check number of valid projections (depth buffers cancels some)
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3Descriptors, PointProjectorPinhole_10000) {
  // ds test configuration
  constexpr size_t number_of_points = 10000;
  constexpr float distance          = 100;
  projection_matrix << 500, 0, 250, 0, 500, 250, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(1771));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(791));

  // ds initialize projector
  PointIntensityDescriptor3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());
  projector_3d.setCameraPose(sensor_poses[0]);

  // ds perform projection
  PointIntensityDescriptor3fVectorCloud points_in_camera;
  PointIntensityDescriptor3fVectorCloud points_in_image;
  std::vector<int> indices;
  const size_t number_of_valid_projections =
    projector_3d.compute(points_in_sensor[0], points_in_camera, points_in_image, indices);

  // ds check number of valid projections (depth buffers cancels some)
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinholeIndexMatrix_100) {
  // ds test configuration
  constexpr size_t number_of_points = 100;
  constexpr float distance          = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(99));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(99));

  // ds initialize projector
  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());

  // tg perform index projection
  Point3fProjectorPinhole::IndexMatrixType index_matrix;
  const size_t number_of_valid_projections =
    projector_3d.compute(index_matrix, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinholeIndexMatrix_1000) {
  // ds test configuration
  constexpr size_t number_of_points = 1000;
  constexpr float distance          = 10;
  projection_matrix << 200, 0, 100, 0, 200, 100, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(494));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(494));

  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());

  // tg perform index projection
  Point3fProjectorPinhole::IndexMatrixType index_matrix;
  const size_t number_of_valid_projections =
    projector_3d.compute(index_matrix, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}

TEST_F(SyntheticWorldSE3, PointProjectorPinholeIndexMatrix_10000) {
  // ds test configuration
  constexpr size_t number_of_points = 10000;
  constexpr float distance          = 100;
  projection_matrix << 500, 0, 250, 0, 500, 250, 0, 0, 1;
  canvas_size << 1000, 1000;

  // ds generate a random world scenario
  const Vector3f mean(distance, distance, distance);
  const Vector3f deviation(distance / 2, distance / 2, distance / 2);
  generateMap(number_of_points, mean, deviation, SensorType::Camera);
  ASSERT_EQ(points_in_sensor[0].size(), static_cast<size_t>(1771));
  ASSERT_EQ(points_in_sensor_projected_in_canvas[0].size(), static_cast<size_t>(791));

  Point3fProjectorPinhole projector_3d;
  projector_3d.setCameraMatrix(projection_matrix);
  projector_3d.setCanvasCols(canvas_size(0));
  projector_3d.setCanvasRows(canvas_size(1));
  projector_3d.param_range_min.setValue(0.1);
  projector_3d.param_range_max.setValue(std::numeric_limits<float>::max());

  // tg perform index projection
  Point3fProjectorPinhole::IndexMatrixType index_matrix;
  const size_t number_of_valid_projections =
    projector_3d.compute(index_matrix, points_in_sensor[0].begin(), points_in_sensor[0].end());

  // ds check number of valid projections
  ASSERT_EQ(number_of_valid_projections, points_in_sensor_projected_in_canvas[0].size());
}
