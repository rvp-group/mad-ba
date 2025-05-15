#include "srrg_pcl/point_intensity_descriptor.h"
#include "srrg_pcl/point_normal.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(PointStatisticsField, DefaultConstruction) {
  PointStatisticsField2D a;
  a.allocate();
  ASSERT_EQ(a.projection().norm(), 0);
  ASSERT_EQ(a.covariance().norm(), 0);
  ASSERT_EQ(a.numberOfOptimizations(), static_cast<size_t>(0));
  ASSERT_FALSE(a.isInlier());

  PointStatisticsField3D b;
  b.allocate();
  ASSERT_EQ(b.projection().norm(), 0);
  ASSERT_EQ(b.covariance().norm(), 0);
  ASSERT_EQ(b.numberOfOptimizations(), static_cast<size_t>(0));
  ASSERT_FALSE(b.isInlier());
}

TEST(PointStatisticsField, Assignment) {
  PointStatisticsField3D a;
  PointStatisticsField3D b;
  b.allocate();
  b.setNumberOfOptimizations(10);
  b.setIsInlier(true);
  b.setProjection(Vector2f(1, 2));
  b.setCovariance(Matrix3f::Ones());
  b.setState(Vector3f(1, 2, 3));

  a = b;

  ASSERT_EQ(a.projection(), b.projection());
  ASSERT_EQ_EIGEN(a.covariance(), b.covariance());
  ASSERT_EQ(a.numberOfOptimizations(), static_cast<size_t>(10));
  ASSERT_TRUE(a.isInlier());
  ASSERT_EQ_EIGEN(a.state(), b.state());
}

TEST(PointStatisticsField, Updates) {
  PointStatisticsField3D a;
  a.allocate();
  ASSERT_EQ(a.state().norm(), 0);
  ASSERT_EQ(a.covariance().norm(), 0);
  ASSERT_EQ(a.numberOfOptimizations(), static_cast<size_t>(0));
  ASSERT_FALSE(a.isInlier());

  for (size_t i = 0; i < 100; ++i) {
    ASSERT_EQ(a.numberOfOptimizations(), i);
    const Vector3f state      = a.state() + Vector3f::Ones();
    const Matrix3f covariance = a.covariance() + Matrix3f::Identity();
    a.addOptimizationResult(state, covariance);
  }
  ASSERT_EQ(a.state()(0), 100);
  ASSERT_EQ(a.state()(1), 100);
  ASSERT_EQ(a.state()(2), 100);
  ASSERT_EQ(a.covariance()(0, 0), 100);
  ASSERT_EQ(a.covariance()(1, 1), 100);
  ASSERT_EQ(a.covariance()(2, 2), 100);
}

TEST(PointStatisticsField, Measurements) {
  PointStatisticsField3D a;
  a.allocate();
  ASSERT_EQ(a.measurements().size(), static_cast<size_t>(0));

  PointStatisticsField3D::CameraMeasurement measurement(
    Vector3f(1, 2, 3), Vector3f(3, 2, 1), Isometry3f::Identity(), Isometry3f::Identity());
  a.addMeasurement(measurement);
  ASSERT_EQ(a.measurements().size(), static_cast<size_t>(1));

  PointStatisticsField3D b = a;
  ASSERT_EQ(b.measurements().size(), static_cast<size_t>(1));
  ASSERT_EQ_EIGEN(b.measurements().back().point_in_image, a.measurements().back().point_in_image);
  ASSERT_EQ_EIGEN(b.measurements().back().point_in_camera, a.measurements().back().point_in_camera);
}
