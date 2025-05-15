#include <iostream>

#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;
using namespace srrg2_core::geometry2d;
using namespace srrg2_core::geometry3d;

#define TOLERANCE 5e-6

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(DummyData, Geometry2D) {
  Vector3f pose;
  pose << 0.3, 0.5, 1.57;

  const Isometry2f iso  = v2t(pose);
  const Vector3f b_pose = t2v(iso);
  ASSERT_NEAR_EIGEN(b_pose, pose, TOLERANCE);

  Vector2f euclidean;
  euclidean << 0.5, -0.5;

  const Vector2f polar               = euclidean2polar(euclidean);
  const Vector2f euclidean_retrieved = polar2euclidean(polar);
  ASSERT_NEAR_EIGEN(euclidean, euclidean_retrieved, TOLERANCE);
}

TEST(DummyData, Geometry3D) {
  Vector6f pose3;
  pose3 << 0.3, 0.5, 0.2, 0.5, -0.5, 0.5;
  const Isometry3f iso3  = v2t(pose3);
  const Vector6f b_pose3 = t2v(iso3);
  ASSERT_NEAR_EIGEN(pose3, b_pose3, TOLERANCE);

  Vector6f ta;
  ta << 0.1, -0.9, 1.1, 1.57, -1.1, 3.01;
  const Isometry3f taiso = ta2t(ta);
  const Vector6f b_ta    = t2ta(taiso);
  ASSERT_NEAR_EIGEN(ta, b_ta, TOLERANCE);

  Vector3f euclidean3;
  euclidean3 << 0.5, 0.6, 10;
  const Vector3f polar3       = euclidean2polar(euclidean3);
  const Vector3f b_euclidean3 = polar2euclidean(polar3);
  ASSERT_NEAR_EIGEN(euclidean3, b_euclidean3, TOLERANCE);

  Vector3f unhomogeneous_v;
  unhomogeneous_v << 0.5, 1.3, 0.8;
  const Vector3f homogeneous_v = hom(unhomogeneous_v);
  ASSERT_EQ(homogeneous_v[0], 0.625);
  ASSERT_EQ(homogeneous_v[1], 1.625);
  ASSERT_EQ(homogeneous_v[2], 1);

  const float x{0.5}, y{0.2}, z{1.2}, theta{0.523599};
  pose3 << x, y, z, 0.0, 0.0, theta;
  const Isometry2f pose_2d = get2dFrom3dPose(ta2t(pose3));
  Vector3f reference_2d_pose;
  reference_2d_pose << x, y, theta;
  ASSERT_EQ_EIGEN(t2v(pose_2d), reference_2d_pose);
}