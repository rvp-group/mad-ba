#include "srrg_pcl/point_intensity_descriptor.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(PointDescriptorFieldCvMatBinary, DefaultConstruction) {
  // ds must be empty
  PointDescriptorFieldCvMatBinary field;
  ASSERT_EQ(field.value.rows, 0);
  ASSERT_EQ(field.value.cols, 0);
  ASSERT_EQ(field.value.data, nullptr);

  PointIntensityDescriptor3f point;
  ASSERT_EQ(point.descriptor().rows, 0);
  ASSERT_EQ(point.descriptor().cols, 0);
  ASSERT_EQ(point.descriptor().data, nullptr);
}

TEST(PointDescriptorFieldCvMatBinary, DirectManipulation) {
  ASSERT_EQ(sizeof(uchar), static_cast<size_t>(1));
  cv::Mat descriptor(1, 2, CV_8UC1); // ds 2 byte descriptor
  descriptor.setTo(0);

  // ds direct assignments
  PointIntensityDescriptor3f point_a;
  point_a.descriptor() = descriptor;
  ASSERT_EQ(point_a.descriptor().at<uchar>(0, 0), 0);
  ASSERT_EQ(point_a.descriptor().at<uchar>(0, 1), 0);

  descriptor.setTo(1);
  PointIntensityDescriptor3f point_b;
  point_b.descriptor() = descriptor;
  ASSERT_EQ(point_b.descriptor().at<uchar>(0, 0), 1);
  ASSERT_EQ(point_b.descriptor().at<uchar>(0, 1), 1);

  // ds high level assignment
  point_a = point_b;
  ASSERT_EQ(point_a.descriptor().at<uchar>(0, 0), 1);
  ASSERT_EQ(point_a.descriptor().at<uchar>(0, 1), 1);
  ASSERT_EQ(point_b.descriptor().at<uchar>(0, 0), 1);
  ASSERT_EQ(point_b.descriptor().at<uchar>(0, 1), 1);

  // ds copy construction
  PointIntensityDescriptor3f point_c(point_a);
  ASSERT_EQ(point_c.descriptor().at<uchar>(0, 0), 1);
  ASSERT_EQ(point_c.descriptor().at<uchar>(0, 1), 1);
}

TEST(PointDescriptorFieldCvMatBinary, IndirectManipulation) {
  ASSERT_EQ(sizeof(uchar), static_cast<size_t>(1));
  cv::Mat descriptor(1, 2, CV_8UC1); // ds 2 byte descriptor
  descriptor.setTo(1);

  // ds direct assignments
  PointIntensityDescriptor3f point_a;
  point_a.descriptor()  = descriptor;
  point_a.coordinates() = Vector3f(1.0f, 2.0f, 3.0f);
  descriptor.setTo(2);
  PointIntensityDescriptor3f point_b;
  point_b.descriptor()  = descriptor;
  point_b.coordinates() = Vector3f(40.0f, 50.0f, 60.0f);

  // ds perform base point operations that affect the coordinates field and the
  // descriptor
  PointIntensityDescriptor3f point_c = point_a + point_b;
  ASSERT_EQ(point_c.coordinates()(0), 41.0f);
  ASSERT_EQ(point_c.coordinates()(1), 52.0f);
  ASSERT_EQ(point_c.coordinates()(2), 63.0f);

  // ds descriptors can not be "added" - currently the last value of the
  // expression is assigned ds TODO enable binary/mean floating point descriptor
  // merging ds TODO the current behavior is not intuitive!
  ASSERT_EQ(point_c.descriptor().at<uchar>(0, 0), 2);
  ASSERT_EQ(point_c.descriptor().at<uchar>(0, 1), 2);

  point_c = point_a - point_b;
  ASSERT_EQ(point_c.coordinates()(0), -39.0f);
  ASSERT_EQ(point_c.coordinates()(1), -48.0f);
  ASSERT_EQ(point_c.coordinates()(2), -57.0f);

  // ds descriptors can not be "subtracted" - currently the last value of the
  // expression is assigned ds TODO enable binary/mean floating point descriptor
  // merging ds TODO the current behavior is not intuitive!
  ASSERT_EQ(point_c.descriptor().at<uchar>(0, 0), 2);
  ASSERT_EQ(point_c.descriptor().at<uchar>(0, 1), 2);
}

TEST(PointDescriptorFieldCvMatBinary, HammingDistance) {
  ASSERT_EQ(sizeof(uchar), static_cast<size_t>(1));
  cv::Mat descriptor_a(1, 2, CV_8UC1);
  descriptor_a.setTo(0);
  cv::Mat descriptor_b(1, 2, CV_8UC1);
  descriptor_b.setTo(255); // ds 2x8=16 bit distance

  PointIntensityDescriptor3f point_a;
  point_a.descriptor() = descriptor_a;
  PointIntensityDescriptor3f point_b;
  point_b.descriptor() = descriptor_b;

  // ds hard comparison - reference
  ASSERT_EQ(
    cv::norm(point_a.descriptor(), point_a.descriptor(), cv::NORM_HAMMING), 0);
  ASSERT_EQ(
    cv::norm(point_b.descriptor(), point_b.descriptor(), cv::NORM_HAMMING), 0);
  ASSERT_EQ(
    cv::norm(point_a.descriptor(), point_b.descriptor(), cv::NORM_HAMMING), 16);

  // ds soft comparison (works with any descriptor type and size)
  ASSERT_EQ(point_a.field<2>().distance(point_a.field<2>()), 0);
  ASSERT_EQ(point_b.field<2>().distance(point_b.field<2>()), 0);
  ASSERT_EQ(point_a.field<2>().distance(point_b.field<2>()), 16);
}

TEST(PointDescriptorFieldBit256, DefaultConstruction) {
  // ds must be empty
  PointDescriptorFieldBit256 field;
  ASSERT_EQ(field.value.count(), static_cast<size_t>(0));
  ASSERT_EQ(field.value.size(), static_cast<size_t>(256));
}

TEST(PointDescriptorFieldBit256, HammingDistance) {
  PointDescriptorFieldBit256 field_a;
  for (size_t i = 0; i < 42; ++i) {
    field_a.value.set(i);
  }
  PointDescriptorFieldBit256 field_b;

  // ds soft comparison (works with any descriptor type and size)
  ASSERT_EQ(field_a.distance(field_a), 0);
  ASSERT_EQ(field_b.distance(field_b), 0);
  ASSERT_EQ(field_a.distance(field_b), 42);
}

TEST(PointDescriptorFieldBit256, ConversionCvToBitset) {
  ASSERT_EQ(sizeof(uchar), static_cast<size_t>(1));
  cv::Mat descriptor_a(1, 32, CV_8UC1);
  descriptor_a.setTo(0);
  cv::Mat descriptor_b(1, 32, CV_8UC1);
  descriptor_b.setTo(255); // ds set all byte blocks to the max

  // ds types to convert to each other
  PointDescriptorFieldCvMatBinary field_cv_a;
  PointDescriptorFieldCvMatBinary field_cv_b;
  PointDescriptorFieldBit256 field_std_a;
  PointDescriptorFieldBit256 field_std_b;

  // ds configure reference types
  field_cv_a.value = descriptor_a;
  field_cv_b.value = descriptor_b;

  // ds convert data
  field_std_a.from(descriptor_a);
  field_std_b.from(descriptor_b);
  ASSERT_EQ(field_std_a.value.count(), static_cast<size_t>(0));
  ASSERT_EQ(field_std_b.value.count(), static_cast<size_t>(256));
  ASSERT_EQ(field_std_a.distance(field_std_b), 256);

  // ds reset
  field_std_a = {};
  field_std_b = {};

  // ds convert data
  field_std_a.from(field_cv_a);
  field_std_b.from(field_cv_b);
  ASSERT_EQ(field_std_a.value.count(), static_cast<size_t>(0));
  ASSERT_EQ(field_std_b.value.count(), static_cast<size_t>(256));
  ASSERT_EQ(field_std_a.distance(field_std_b), 256);
}
