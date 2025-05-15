#include <iostream>
#include <srrg_system_utils/system_utils.h>
//#include <srrg_test/test_helper.hpp>
#include <srrg_matchable/matchable.h>
#include <srrg_matchable/visual_matchable.h>

#include <gtest/gtest.h>
using namespace srrg2_core;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//! ------------------------------------------------------------------------ //
//! ------------------------------------------------------------------------ //
//! ------------------------------------------------------------------------ //
TEST(DUMMY_DATA, MatchableBasics) {
  const Matchablef::Type type_float = Matchablef::Type::Line;
  const Vector3f origin_float       = Vector3f::Random();
  const Vector3f direction_float    = Vector3f::UnitY();
  Matchablef m_float(type_float, origin_float);
  m_float.setDirection(direction_float);

  ASSERT_FLOAT_EQ(type_float, m_float.type());
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(origin_float(i), m_float.origin()(i));
    ASSERT_FLOAT_EQ(direction_float(i), m_float.direction()(i));
  }

  const Matchabled::Type type_double = Matchabled::Type::Plane;
  const Vector3d origin_double       = Vector3d::Random();
  const Matrix3d rotation_double =
    Eigen::AngleAxisd(0.25 * M_PI, Vector3d::UnitY()).toRotationMatrix();
  const Vector3d direction_double = rotation_double.col(0);
  Matchabled m_double(type_double, origin_double, rotation_double);

  ASSERT_FLOAT_EQ(type_double, m_double.type());
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(origin_double(i), m_double.origin()(i));
    ASSERT_FLOAT_EQ(direction_double(i), m_double.direction()(i));
  }

  Matchablef::FullVectorType vectorized_matchable = Matchablef::FullVectorType::Zero();
  vectorized_matchable                            = m_float.toVector();
  ASSERT_FLOAT_EQ((float) type_float, vectorized_matchable[0]);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(origin_float(i), vectorized_matchable[i + 1]);
    ASSERT_FLOAT_EQ(direction_float(i), vectorized_matchable[i + 4]);
  }

  Matchablef m_float_copy;
  m_float_copy.fromVector(vectorized_matchable);
  ASSERT_FLOAT_EQ(type_float, m_float_copy.type());
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(origin_float(i), m_float_copy.origin()(i));
    ASSERT_FLOAT_EQ(direction_float(i), m_float_copy.direction()(i));
  }

  Matchablef m_float_assignment_0(MatchableBase::Type::Plane, Vector3f::Random());
  const Vector3f identity_direction = m_float_assignment_0.direction();
  const Vector3f zero_origin        = Vector3f::Zero();
  m_float_assignment_0.setDirection(direction_float);

  // ia assign to the identity
  m_float_assignment_0 = Matchablef::Identity();
  // ai copy ctor
  Matchablef m_float_assignment_1(m_float_assignment_0);

  ASSERT_FLOAT_EQ(MatchableBase::Type::Point, m_float_assignment_0.type());
  ASSERT_FLOAT_EQ(MatchableBase::Type::Point, m_float_assignment_1.type());
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(zero_origin(i), m_float_assignment_0.origin()(i));
    ASSERT_FLOAT_EQ(identity_direction(i), m_float_assignment_0.direction()(i));

    ASSERT_FLOAT_EQ(zero_origin(i), m_float_assignment_1.origin()(i));
    ASSERT_FLOAT_EQ(identity_direction(i), m_float_assignment_1.direction()(i));
  }
}

//! ------------------------------------------------------------------------ //
//! ------------------------------------------------------------------------ //
//! ------------------------------------------------------------------------ //
TEST(DUMMY_DATA, MatchableOperators) {
  Matrix3f rot_y       = AngleAxisf(M_PI, Vector3f::UnitY()).toRotationMatrix();
  Matrix3f rot_z       = AngleAxisf(M_PI, Vector3f::UnitZ()).toRotationMatrix();
  Vector3f translation = Vector3f(1.0, 0.0, 0.0);
  Matchablef matchable_p0(MatchableBase::Type::Point);
  Matchablef matchable_l0(MatchableBase::Type::Line, translation, rot_y);

  Isometry3f T    = Isometry3f::Identity();
  T.linear()      = rot_z;
  T.translation() = translation;

  // ia transform
  Matchablef matchable_p1              = matchable_p0.transform(T);
  const Vector3f expected_origin_p1    = rot_z * matchable_p0.origin() + translation;
  const Vector3f expected_direction_p1 = matchable_p0.direction();
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_p1(i), matchable_p1.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_p1(i), matchable_p1.direction()(i));
  }

  // ia transform in place w/ point
  matchable_p0.transformInPlace(T);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_p1(i), matchable_p0.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_p1(i), matchable_p0.direction()(i));
  }

  // ia transform
  Matchablef matchable_l1              = matchable_l0.transform(T);
  const Vector3f expected_origin_l1    = rot_z * matchable_l0.origin() + translation;
  const Vector3f expected_direction_l1 = (rot_z * rot_y).col(0);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_l1(i), matchable_l1.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l1(i), matchable_l1.direction()(i));
  }

  // ia transform in place w/ line
  matchable_l0.transformInPlace(T);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_l1(i), matchable_l0.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l1(i), matchable_l0.direction()(i));
  }
}

//! ------------------------------------------------------------------------ //
//! ------------------------------------------------------------------------ //
//! ------------------------------------------------------------------------ //
TEST(DUMMY_DATA, VisualMatchable) {
  const MatchableBase::Type type_point = MatchableBase::Type::Point;
  const Vector3f origin_point          = Vector3f::Random();
  const Vector3f direction_point       = Vector3f::UnitY();

  const MatchableBase::Type type_line = MatchableBase::Type::Line;
  const Vector3f origin_line          = Vector3f::Random();
  const Matrix3f rotation_line =
    Eigen::AngleAxisf(0.25 * M_PI, Vector3f::UnitY()).toRotationMatrix();
  const Vector3f direction_line = rotation_line.col(0);

  // ia basics
  VisualMatchablef m_point(type_point, origin_point);
  m_point.setDirection(direction_point);
  ASSERT_FLOAT_EQ(type_point, m_point.type());
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(origin_point(i), m_point.origin()(i));
    ASSERT_FLOAT_EQ(direction_point(i), m_point.direction()(i));
  }

  VisualMatchablef m_line(type_line, origin_line, rotation_line);
  ASSERT_FLOAT_EQ(type_line, m_line.type());
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(origin_line(i), m_line.origin()(i));
    ASSERT_FLOAT_EQ(direction_line(i), m_line.direction()(i));
  }

  // ia transformation
  const Matrix3f rot_y       = AngleAxisf(M_PI, Vector3f::UnitY()).toRotationMatrix();
  const Matrix3f rot_z       = AngleAxisf(M_PI, Vector3f::UnitZ()).toRotationMatrix();
  const Vector3f translation = Vector3f(1.0, 0.0, 0.0);

  VisualMatchablef m_l0(MatchableBase::Type::Line, translation, rot_y);

  Isometry3f T    = Isometry3f::Identity();
  T.linear()      = rot_z;
  T.translation() = translation;

  // ia transform and operators
  VisualMatchablef m_l1                = m_l0.transform(T);
  const Vector3f expected_origin_l1    = rot_z * m_l0.origin() + translation;
  const Vector3f expected_direction_l1 = (rot_z * rot_y).col(0);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_l1(i), m_l1.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l1(i), m_l1.direction()(i));
  }

  // ia transform in place w/ line
  m_l0.transformInPlace(T);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_l1(i), m_l0.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l1(i), m_l0.direction()(i));
  }

  VisualMatchablef m_l2(m_l0);
  // ia should fall back to unstable situation and return [0 0 1]
  VisualMatchablef m_l3                = m_l2 - m_l0;
  const Vector3f expected_direction_l3 = Vector3f::UnitZ();
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(0.0f, m_l3.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l3(i), m_l3.direction()(i));
  }

  // ia still operators
  VisualMatchablef m_l4(m_l0);
  VisualMatchablef m_l5 = m_l4 + m_l0;
  VisualMatchablef m_l6 = m_l0 * 2.0f;

  const Vector3f expected_origin_l5    = m_l0.origin() + m_l0.origin();
  const Vector3f expected_direction_l5 = (m_l0.direction() + m_l0.direction()).normalized();

  const Vector3f expected_origin_l6    = m_l0.origin() * 2.0f;
  const Vector3f expected_direction_l6 = m_l0.direction();
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_l5(i), m_l5.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l5(i), m_l5.direction()(i));

    ASSERT_FLOAT_EQ(expected_origin_l6(i), m_l6.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l6(i), m_l6.direction()(i));
  }

  m_l4 *= 2.0f;
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(expected_origin_l6(i), m_l4.origin()(i));
    ASSERT_FLOAT_EQ(expected_direction_l6(i), m_l4.direction()(i));
  }

  // ia transformation also of the support cloud
  VisualMatchablef m_pl0(MatchableBase::Type::Plane, Vector3f::Zero());
  const size_t target_support_size = 10000;
  m_pl0.support().reserve(target_support_size);
  for (size_t k = 0; k < target_support_size; ++k) {
    VisualMatchablef::ExtentType support_p;
    support_p.coordinates() = Vector3f::Ones() * k;
    m_pl0.support().emplace_back(support_p);
  }
  ASSERT_EQ(static_cast<size_t>(m_pl0.support().size()), target_support_size);

  // ia fake transform
  VisualMatchablef m_pl0_transf = m_pl0.transformSupport(Isometry3f::Identity());
  ASSERT_EQ(static_cast<size_t>(m_pl0_transf.support().size()), target_support_size);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(m_pl0.origin()(i), m_pl0_transf.origin()(i));
    ASSERT_FLOAT_EQ(m_pl0.direction()(i), m_pl0_transf.direction()(i));
  }

  for (size_t i = 0; i < target_support_size; ++i) {
    const auto& p   = m_pl0.support()[i];
    const auto& p_t = m_pl0_transf.support()[i];
    for (size_t i = 0; i < 3; ++i) {
      ASSERT_FLOAT_EQ(p.coordinates()(i), p_t.coordinates()(i));
      ASSERT_FLOAT_EQ(p.normal()(i), p_t.normal()(i));
    }
    ASSERT_FLOAT_EQ(p.curvature(), p_t.curvature());
  }

  // ia actual transform
  VisualMatchablef m_pl0_transf_2 = m_pl0.transformSupport(T);
  ASSERT_EQ(static_cast<size_t>(m_pl0_transf_2.support().size()), target_support_size);
  const Vector3f expected_orgin_m_pl0_transf_2     = rot_z * m_pl0.origin() + translation;
  const Vector3f expected_direction_m_pl0_transf_2 = (rot_z * m_pl0.rotation()).col(0);
  for (size_t i = 0; i < 3; ++i) {
    ASSERT_FLOAT_EQ(m_pl0_transf_2.origin()(i), expected_orgin_m_pl0_transf_2(i));
    ASSERT_FLOAT_EQ(m_pl0_transf_2.direction()(i), expected_direction_m_pl0_transf_2(i));
  }

  for (size_t i = 0; i < target_support_size; ++i) {
    const Vector3f ref_coord = T * m_pl0.support()[i].coordinates();
    const Vector3f ref_dir   = T.linear() * m_pl0.support()[i].normal();
    const auto ref_curvature = m_pl0.support()[i].curvature();
    const auto& p_t          = m_pl0_transf_2.support()[i];
    for (size_t i = 0; i < 3; ++i) {
      ASSERT_FLOAT_EQ(ref_coord(i), p_t.coordinates()(i));
      ASSERT_FLOAT_EQ(ref_dir(i), p_t.normal()(i));
    }
    ASSERT_FLOAT_EQ(ref_curvature, p_t.curvature());
  }
}
