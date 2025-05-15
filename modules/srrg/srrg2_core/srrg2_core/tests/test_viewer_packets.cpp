#include "srrg_pcl/instances.h"
#include "srrg_test/test_helper.hpp"
#include "srrg_viewer/viewer_core/packets.h"

using namespace srrg2_core;

static constexpr uint32_t max_buffer_size = 1024 * 1024 * 10;

int main(int argc, char** argv) {
  return srrg2_test::runTests(argc, argv);
}

TEST(Viewer, PacketSerialization) {
  // ds TODO split this test into individual packets
  const size_t num_points = 20;
  Vector3f* points        = new Vector3f[num_points];
  Vector3f* normals       = new Vector3f[num_points];

  for (size_t i = 0; i < num_points; ++i) {
    points[i]  = Vector3f::Ones() * i;
    normals[i] = points[i];
  }

  PointNormalColor3fVectorCloud point_vector;
  PointIntensityDescriptor2fVectorCloud pd_vector;
  point_vector.resize(num_points);
  pd_vector.resize(num_points);
  for (size_t i = 0; i < point_vector.size(); ++i) {
    point_vector[i].coordinates().setRandom();
    point_vector[i].normal().setOnes();
    point_vector[i].color().setRandom();

    pd_vector[i].coordinates().setRandom();
  }

  // ia matchables
  const size_t num_matchables = 20;
  MatchablefVector matchable_vector;
  matchable_vector.reserve(num_matchables);
  VisualMatchablefVector visual_matchable_vector;
  visual_matchable_vector.reserve(num_matchables);
  for (size_t m = 0; m < num_matchables; ++m) {
    Vector3f origin = Vector3f::Zero() + Vector3f::Ones() * m;
    std::cerr << "origin = " << origin.transpose() << std::endl;

    Matchablef matchable(MatchableBase::Type::Plane, origin);
    matchable_vector.emplace_back(matchable);

    VisualMatchablef v_matchable(MatchableBase::Type::Plane, origin);
    v_matchable.color() = ColorPalette::color3fBlue();
    v_matchable.support().reserve(1000);
    for (size_t i = 0; i < 1000; ++i) {
      PointNormalCurvatureColor3f extent_point;
      extent_point.coordinates() = Vector3f::Random();
      extent_point.normal()      = Vector3f::UnitX();
      extent_point.color()       = v_matchable.color();
      extent_point.curvature()   = 0.1f;
      v_matchable.support().emplace_back(extent_point);
    }

    visual_matchable_vector.emplace_back(v_matchable);
  }

  char* buffer_start = new char[max_buffer_size];

  std::cerr << "buffer_start: " << (size_t) buffer_start << std::endl << std::endl;

  // ia payload packets
  PacketPayloadPoints p_pack(num_points, points);
  PacketPayloadPoints pn_pack(num_points, points, normals);

  // ia attribute packet
  PacketAttributeColorRGBA color_pack(Vector4f(1, 0, 0, 1));
  PacketAttributePointSize p_size_pack(2.0f);

  // ia command packet
  PacketCommandPushColor push_color_pack;
  PacketCommandPopAttribute pop_color_attrib;

  // ia object packet
  PacketObjectSphere sphere_pack(2.0f);
  PacketObjectPyramidWireframe pyr_wf_pack(Vector2f(2.0f, 1.0f));

  // ia transform packet
  PacketTransformMultMatrix transf_pack(Matrix4f::Random());

  // ia pointcloud packet
  PacketPointNormalColor3fVectorCloud point_cloud_pack(&point_vector);
  PacketPointIntensityDescriptor2fVectorCloud point_intensity_cloud_pack(&pd_vector);

  // ia machable vector
  PacketMatchablefVector matchable_pack(&matchable_vector);

  // ia visual matchable vector
  PacketVisualMatchablefVector visual_matchable_pack(&visual_matchable_vector);

  // ia info packet
  PacketInfoEndEpoch end_pack;

  // ia serialize
  char* buffer_end = buffer_start;

  buffer_end = p_pack.serialize(buffer_end);
  std::cerr << "buffer_end 01: " << (size_t) buffer_end << std::endl;
  buffer_end = pn_pack.serialize(buffer_end);
  std::cerr << "buffer_end 02: " << (size_t) buffer_end << std::endl;
  buffer_end = color_pack.serialize(buffer_end);
  std::cerr << "buffer_end 03: " << (size_t) buffer_end << std::endl;
  buffer_end = p_size_pack.serialize(buffer_end);
  std::cerr << "buffer_end 04: " << (size_t) buffer_end << std::endl;
  buffer_end = push_color_pack.serialize(buffer_end);
  std::cerr << "buffer_end 05: " << (size_t) buffer_end << std::endl;
  buffer_end = pop_color_attrib.serialize(buffer_end);
  std::cerr << "buffer_end 06: " << (size_t) buffer_end << std::endl;
  buffer_end = sphere_pack.serialize(buffer_end);
  std::cerr << "buffer_end 07: " << (size_t) buffer_end << std::endl;
  buffer_end = pyr_wf_pack.serialize(buffer_end);
  std::cerr << "buffer_end 08: " << (size_t) buffer_end << std::endl;
  buffer_end = transf_pack.serialize(buffer_end);
  std::cerr << "buffer_end 09: " << (size_t) buffer_end << std::endl;
  buffer_end = point_cloud_pack.serialize(buffer_end);
  std::cerr << "buffer_end 10: " << (size_t) buffer_end << std::endl;
  buffer_end = point_intensity_cloud_pack.serialize(buffer_end);
  std::cerr << "buffer_end 11: " << (size_t) buffer_end << std::endl;
  buffer_end = matchable_pack.serialize(buffer_end);
  std::cerr << "buffer_end 12: " << (size_t) buffer_end << std::endl;
  buffer_end = visual_matchable_pack.serialize(buffer_end);
  std::cerr << "buffer_end 13: " << (size_t) buffer_end << std::endl;

  buffer_end = end_pack.serialize(buffer_end);
  std::cerr << "buffer_end END_PACK: " << (size_t) buffer_end << std::endl;
  std::cerr << std::endl << std::endl;

  // ia deserialize
  PacketPayloadPoints p_recv_pack;
  PacketPayloadPoints pn_recv_pack;
  PacketAttributeColorRGBA color_recv_pack(Vector4f::Zero());
  PacketAttributePointSize p_size_recv_pack(0.0);
  PacketCommandPushColor push_color_recv_pack;
  PacketCommandPopAttribute pop_attrib_recv_pack;
  PacketObjectSphere sphere_recv_pack(0.0f);
  PacketObjectPyramidWireframe pyr_wf_recv_pack(Vector2f::Zero());
  PacketTransformMultMatrix transf_racv_pack(Matrix4f::Zero());
  PacketPointNormalColor3fVectorCloud point_cloud_recv_pack;
  PacketPointIntensityDescriptor2fVectorCloud point_intensity_cloud_recv_pack;
  PacketMatchablefVector matchable_recv_pack;
  PacketVisualMatchablefVector visual_matchable_recv_pack;
  PacketInfoEndEpoch end_recv_pack;

  const char* b = buffer_start;
  b             = p_recv_pack.deserialize(b);
  std::cerr << "buffer_end 01: " << (size_t) b << std::endl;
  b = pn_recv_pack.deserialize(b);
  std::cerr << "buffer_end 02: " << (size_t) b << std::endl;
  b = color_recv_pack.deserialize(b);
  std::cerr << "buffer_end 03: " << (size_t) b << std::endl;
  b = p_size_recv_pack.deserialize(b);
  std::cerr << "buffer_end 04: " << (size_t) b << std::endl;
  b = push_color_recv_pack.deserialize(b);
  std::cerr << "buffer_end 05: " << (size_t) b << std::endl;
  b = pop_attrib_recv_pack.deserialize(b);
  std::cerr << "buffer_end 06: " << (size_t) b << std::endl;
  b = sphere_recv_pack.deserialize(b);
  std::cerr << "buffer_end 07: " << (size_t) b << std::endl;
  b = pyr_wf_recv_pack.deserialize(b);
  std::cerr << "buffer_end 08: " << (size_t) b << std::endl;
  b = transf_racv_pack.deserialize(b);
  std::cerr << "buffer_end 09: " << (size_t) b << std::endl;
  b = point_cloud_recv_pack.deserialize(b);
  std::cerr << "buffer_end 10: " << (size_t) b << std::endl;
  b = point_intensity_cloud_recv_pack.deserialize(b);
  std::cerr << "buffer_end 11: " << (size_t) b << std::endl;
  b = matchable_recv_pack.deserialize(b);
  std::cerr << "buffer_end 12: " << (size_t) b << std::endl;
  b = visual_matchable_recv_pack.deserialize(b);
  std::cerr << "buffer_end 13: " << (size_t) b << std::endl;

  b = end_recv_pack.deserialize(b);
  std::cerr << "buffer_end END_PACK: " << (size_t) b << std::endl;
  std::cerr << std::endl << std::endl;

  for (const auto& p : *point_cloud_recv_pack.data_vector) {
    std::cerr << "point recv = " << p.coordinates().transpose() << std::endl;
  }

  for (const auto& m : *matchable_recv_pack.data_vector) {
    std::cerr << "matchbale recv = " << m.origin().transpose() << std::endl;
  }

  for (const auto& p : *point_intensity_cloud_recv_pack.data_vector) {
    std::cerr << "point recv = " << p.coordinates().transpose() << std::endl;
  }

  delete[] buffer_start;
  delete[] points;
  delete[] normals;
}

// ia cv image packets
TEST(ViewerPacketSerialization, PacketCvMat) {
  // ds input images
  cv::Mat example_image(100, 100, CV_8U);
  cv::randu(example_image, cv::Scalar(0), cv::Scalar(100));
  cv::Mat example_image_color(100, 100, CV_8UC3);
  cv::randu(example_image_color, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100));

  // ds stream buffer
  char* buffer = new char[max_buffer_size];
  PacketInfoEndEpoch end_pack;

  // ds write
  PacketCvMat image_packet_write(example_image);
  PacketCvMat image_color_packet_write(example_image_color);
  char* write_buffer = buffer;
  write_buffer       = image_packet_write.serialize(write_buffer);
  write_buffer       = image_color_packet_write.serialize(write_buffer);
  end_pack.serialize(write_buffer);

  // ds read
  PacketCvMat image_packet_read;
  PacketCvMat image_color_packet_read;
  const char* read_buffer = buffer;
  read_buffer             = image_packet_read.deserialize(read_buffer);
  read_buffer             = image_color_packet_read.deserialize(read_buffer);

  // ds free buffer
  delete buffer;

  // ds verify symmetric serialization
  ASSERT_EQ(image_packet_read.data.type(), example_image.type());
  ASSERT_EQ(image_packet_read.data.rows, example_image.rows);
  ASSERT_EQ(image_packet_read.data.cols, example_image.cols);
  for (int r = 0; r < image_packet_read.data.rows; ++r) {
    for (int c = 0; c < image_packet_read.data.cols; ++c) {
      ASSERT_EQ(image_packet_read.data.at<uchar>(r, c), example_image.at<uchar>(r, c));
    }
  }
  ASSERT_EQ(image_color_packet_read.data.type(), example_image_color.type());
  ASSERT_EQ(image_color_packet_read.data.rows, example_image_color.rows);
  ASSERT_EQ(image_color_packet_read.data.cols, example_image_color.cols);
  for (int r = 0; r < image_color_packet_read.data.rows; ++r) {
    for (int c = 0; c < image_color_packet_read.data.cols; ++c) {
      ASSERT_EQ(image_color_packet_read.data.at<uchar>(r, c), example_image_color.at<uchar>(r, c));
    }
  }
}
