#include <srrg_pcl/point.h>
#include <srrg_pcl/point_color.h>
#include <srrg_pcl/point_intensity.h>
#include <srrg_pcl/point_normal.h>
#include <srrg_pcl/point_normal_color.h>
#include <srrg_pcl/point_normal_intensity.h>

#include <srrg_viewer/viewer_core/color_palette.h>

#include <gtest/gtest.h>

#define SRRG_ABS_ERR 1e-6

using namespace srrg2_core;

TEST(srrg_pcl, Point) {
  Point2f p2f0; p2f0.coordinates() = Vector2f(0.1, -3.0);
  Point2f p2f1; p2f1.coordinates() = Vector2f(1.0, 2.5);
  Point2f mean = (p2f0 + p2f1) * .5f;
  mean.normalize();
  EXPECT_FLOAT_EQ(Vector2f(0.55, -0.25).sum(), mean.coordinates().sum());
}

TEST(srrg_pcl, PointColor) {
  PointColor3f pc3f0; pc3f0.coordinates() = Vector3f(0.1, -3.0, 1.0);
  pc3f0.color() = ColorPalette::color3fBlack();
  PointColor3f pc3f1; pc3f1.coordinates() = Vector3f(1.0, 2.5, -1.2);  
  pc3f1.color() = ColorPalette::color3fBlack();
  PointColor3f mean = (pc3f0 + pc3f1) * .5f;
  mean.normalize();
  EXPECT_FLOAT_EQ(Vector3f(0.55, -0.25, -0.1).sum(), mean.coordinates().sum());
  EXPECT_FLOAT_EQ(ColorPalette::color3fBlack().sum(), mean.color().sum());
}

TEST(srrg_pcl, PointIntensity) {
  PointIntensity3f pi3f0; pi3f0.coordinates() = Vector3f(0.1, -3.0, 1.0);
  pi3f0.intensity() = 0.3;
  PointIntensity3f pi3f1; pi3f1.coordinates() = Vector3f(1.0, 2.5, -1.2);  
  pi3f1.intensity() = 0.3;
  PointIntensity3f mean = (pi3f0 + pi3f1) * .5f;
  mean.normalize();
  EXPECT_FLOAT_EQ(Vector3f(0.55, -0.25, -0.1).sum(), mean.coordinates().sum());
  EXPECT_FLOAT_EQ(0.3, mean.intensity());
}

TEST(srrg_pcl, PointNormal) {
  PointNormal3f pn3f0; 
  pn3f0.coordinates() = Vector3f(0.1, -3.0, 1.0);
  pn3f0.normal() = Vector3f(0.72984, 0.17078, 0.66194); // from octave 
  PointNormal3f pn3f1; pn3f1.coordinates() = Vector3f(1.0, 2.5, -1.2);  
  pn3f1.normal() = Vector3f(0.66836, 0.68408, 0.29211); // from octave
  PointNormal3f mean = (pn3f0 + pn3f1) * .5f;
  mean.normalize();
  EXPECT_FLOAT_EQ(Vector3f(0.55, -0.25, -0.1).sum(), mean.coordinates().sum());
  EXPECT_FLOAT_EQ(Vector3f(0.73733, 0.450804, 0.503111).sum(), mean.normal().sum());
  EXPECT_FLOAT_EQ(1.f, mean.normal().norm());
}

TEST(srrg_pcl, PointNormalColor) {
  PointNormalColor3f pnc3f0; 
  pnc3f0.coordinates() = Vector3f(0.1, -3.0, 1.0);
  pnc3f0.normal() = Vector3f(0.72984, 0.17078, 0.66194); // from octave 
  pnc3f0.color() = ColorPalette::color3fDarkYellow();
  PointNormalColor3f pnc3f1; pnc3f1.coordinates() = Vector3f(1.0, 2.5, -1.2);  
  pnc3f1.normal() = Vector3f(0.66836, 0.68408, 0.29211); // from octave
  pnc3f1.color() = ColorPalette::color3fDarkYellow();
  PointNormalColor3f mean = (pnc3f0 + pnc3f1) * .5f;
  mean.normalize();
  EXPECT_FLOAT_EQ(Vector3f(0.55, -0.25, -0.1).sum(), mean.coordinates().sum());
  EXPECT_FLOAT_EQ(Vector3f(0.73733, 0.450804, 0.503111).sum(), mean.normal().sum());
  EXPECT_FLOAT_EQ(ColorPalette::color3fDarkYellow().sum(), mean.color().sum());
  EXPECT_FLOAT_EQ(1.f, mean.normal().norm());
}

TEST(srrg_pcl, PointNormalIntensity) {
  PointNormalIntensity3f pni3f0; 
  pni3f0.coordinates() = Vector3f(0.1, -3.0, 1.0);
  pni3f0.normal() = Vector3f(0.72984, 0.17078, 0.66194); // from octave 
  pni3f0.intensity() = 0.5;
  PointNormalIntensity3f pni3f1; pni3f1.coordinates() = Vector3f(1.0, 2.5, -1.2);  
  pni3f1.normal() = Vector3f(0.66836, 0.68408, 0.29211); // from octave
  pni3f1.intensity() = 0.5;
  PointNormalIntensity3f mean = (pni3f0 + pni3f1) * .5f;
  mean.normalize();
  EXPECT_FLOAT_EQ(Vector3f(0.55, -0.25, -0.1).sum(), mean.coordinates().sum());
  EXPECT_FLOAT_EQ(Vector3f(0.73733, 0.450804, 0.503111).sum(), mean.normal().sum());
  EXPECT_FLOAT_EQ(.5f, mean.intensity());
  EXPECT_FLOAT_EQ(1.f, mean.normal().norm()); 
}

int main(int argc_, char** argv_) {
  testing::InitGoogleTest(&argc_, argv_);

  auto gtest_output = RUN_ALL_TESTS();

  return gtest_output;
}