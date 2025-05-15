#include "srrg_pcl/point_unprojector_types.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

// ds test fixture
class Point3fUnprojector : public ::testing::Test {
protected:
  void SetUp() override {
    _camera_calibration_matrix << 100, 0, 50, 0, 100, 50, 0, 0, 1;
  }

  template <typename PointVectorCloudType_>
  void generate(PointVectorCloudType_& points_in_camera_frame_, PointVectorCloudType_& features_) {
    points_in_camera_frame_.resize(3);
    points_in_camera_frame_[0].coordinates() =
      typename PointVectorCloudType_::PointType::VectorType(-0.5f, -0.5, 1.0); // ds top left
    points_in_camera_frame_[1].coordinates() =
      typename PointVectorCloudType_::PointType::VectorType(0.0f, 0.0, 5.0); // ds camera center
    points_in_camera_frame_[2].coordinates() =
      typename PointVectorCloudType_::PointType::VectorType(0.5, 0.5, 1.0); // ds bottom right

    // ds features to unproject in camera frame: image coordinates + depth
    features_.resize(3);
    for (size_t i = 0; i < 3; ++i) {
      features_[i].coordinates() =
        _camera_calibration_matrix * points_in_camera_frame_[i].coordinates();
      features_[i].coordinates() /= features_[i].coordinates()(2);

      // ds restore the depth information
      features_[i].coordinates()(2) = points_in_camera_frame_[i].coordinates()(2);
    }
    ASSERT_EQ(features_.size(), points_in_camera_frame_.size());
  }

  void TearDown() override {
  }

protected:
  Matrix3f _camera_calibration_matrix = Matrix3f::Identity();
};

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST_F(Point3fUnprojector, SparseUnprojectionPinholePoint3f) {
  // ds allocate a default unprojector and configure it
  Point3fUnprojectorPinhole unprojector;
  unprojector.setCameraMatrix(_camera_calibration_matrix);

  // ds point in camera frame (reference)
  Point3fVectorCloud points_in_camera_frame_reference;

  // ds features to unproject in camera frame: image coordinates + depth
  Point3fVectorCloud features;

  // ds generate world
  generate<Point3fVectorCloud>(points_in_camera_frame_reference, features);

  // ds compute points in camera frame
  Point3fVectorCloud points_in_camera_frame;
  unprojector.compute(features, points_in_camera_frame);
  ASSERT_EQ(points_in_camera_frame.size(), features.size());
  for (size_t i = 0; i < points_in_camera_frame.size(); ++i) {
    ASSERT_NEAR_EIGEN(points_in_camera_frame[i].coordinates(),
                      points_in_camera_frame_reference[i].coordinates(),
                      std::numeric_limits<float>::epsilon());
  }
}

TEST_F(Point3fUnprojector, SparseUnprojectionPinholePointNormal3f) {
  // ds allocate a default unprojector and configure it
  PointNormal3fUnprojectorPinhole unprojector;
  unprojector.setCameraMatrix(_camera_calibration_matrix);

  // ds point in camera frame (reference)
  PointNormal3fVectorCloud points_in_camera_frame_reference;

  // ds features to unproject in camera frame: image coordinates + depth
  PointNormal3fVectorCloud features;

  // ds generate world
  generate<PointNormal3fVectorCloud>(points_in_camera_frame_reference, features);

  // ds populate normals
  for (size_t i = 0; i < features.size(); ++i) {
    ASSERT_EQ_EIGEN(features[i].normal(), Vector3f(0, 0, 0));
    features[i].normal() = Vector3f(1, 0, 0);
  }

  // ds compute points in camera frame
  PointNormal3fVectorCloud points_in_camera_frame;
  unprojector.compute(features, points_in_camera_frame);
  ASSERT_EQ(points_in_camera_frame.size(), features.size());
  for (size_t i = 0; i < points_in_camera_frame.size(); ++i) {
    ASSERT_NEAR_EIGEN(points_in_camera_frame[i].coordinates(),
                      points_in_camera_frame_reference[i].coordinates(),
                      std::numeric_limits<float>::epsilon());

    // ds must be equal, we did not manipulate the float
    ASSERT_EQ_EIGEN(points_in_camera_frame[i].normal(), features[i].normal());
  }
}

TEST_F(Point3fUnprojector, SparseUnprojectionPinholePointIntensityDescriptor3f) {
  // ds allocate a default unprojector and configure it
  PointIntensityDescriptor3fUnprojectorPinhole unprojector;
  unprojector.setCameraMatrix(_camera_calibration_matrix);

  // ds point in camera frame (reference)
  PointIntensityDescriptor3fVectorCloud points_in_camera_frame_reference;

  // ds features to unproject in camera frame: image coordinates + depth
  PointIntensityDescriptor3fVectorCloud features;

  // ds generate world
  generate<PointIntensityDescriptor3fVectorCloud>(points_in_camera_frame_reference, features);

  // ds populate intensity and descriptor fields in features
  // ds the field values must be carried along when unprojecting
  for (size_t i = 0; i < features.size(); ++i) {
    features[i].intensity()  = static_cast<uchar>(i + 10);
    features[i].descriptor() = cv::Mat(i + 10, 0, CV_8UC1);
  }

  // ds compute points in camera frame
  PointIntensityDescriptor3fVectorCloud points_in_camera_frame;
  unprojector.compute(features, points_in_camera_frame);
  ASSERT_EQ(points_in_camera_frame.size(), features.size());
  for (size_t i = 0; i < points_in_camera_frame.size(); ++i) {
    ASSERT_NEAR_EIGEN(points_in_camera_frame[i].coordinates(),
                      points_in_camera_frame_reference[i].coordinates(),
                      std::numeric_limits<float>::epsilon());

    // ds must be equal, we did not manipulate the floats
    ASSERT_EQ(static_cast<int>(points_in_camera_frame[i].intensity()),
              static_cast<int>(features[i].intensity()));
    ASSERT_EQ(points_in_camera_frame[i].descriptor().rows, features[i].descriptor().rows);
  }
}
