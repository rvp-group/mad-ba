#include <srrg_system_utils/system_utils.h>
#include <srrg_boss/id_context.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/laser_message.h>
#include <srrg_messages/imu_message.h>
#include <srrg_messages/joint_state_message.h>
#include <srrg_messages/image_message.h>
#include <srrg_messages/pose_message.h>
#include <srrg_messages/range_message.h>
#include <srrg_messages/static_transform_tree_message.h>

using namespace srrg2_core;

const char* banner[] = { "srrg_message_example: testing messages ",
    "usage: srrg_message_example <file>", 0 };

int main(int argc, char** argv) {

  const size_t laser_size = 10;
  const size_t range_size = 8;

  if (argc == 2) {
    std::cerr << "Write file to " << argv[1] << std::endl;
  } else {
    printBanner(banner);
    return 0;
  }

  SerializationContext* context = new SerializationContext();
  Serializer* ser = new Serializer(context);
  ser->serializationContext()->setOutputFilePath(argv[1]);

  std::cerr << "Init program" << std::endl;
  std::cerr << "Creating laser message" << std::endl;

  std::vector<float> intensities(laser_size);
  std::vector<float> ranges(laser_size);
  float value = 0;
  for (int i = 0; i < laser_size; ++i) {
    intensities[i] = value * 50;
    ranges[i] = value * 100;
    value += 1 / static_cast<float>(laser_size);
  }
  int seq = 10;
  double timestamp = getTime();

  LaserMessage* laser_message = new LaserMessage("/scan", "/scan", seq,
      timestamp);
  Isometry3f offset;
  Matrix4f tmp;
  tmp << 0, 1, 0, 3, -1, 0, 0, 2, 0, 0, 1, 1, 0, 0, 0, 1;
  offset.linear() = tmp.block<3, 3>(0, 0);
  offset.translation() = tmp.block<3, 1>(0, 3);
  laser_message->angle_increment.setValue(0.3);
  laser_message->intensities.setValue(intensities);
  laser_message->min_range.setValue(0);
  laser_message->min_angle.setValue(0);
  laser_message->max_angle.setValue(100);
  laser_message->max_range.setValue(100);
  laser_message->ranges.setValue(ranges);
  laser_message->scan_time.setValue(getTime());
  laser_message->time_increment.setValue(0.001);

  std::cerr << "Serializing LaserMessage" << std::endl;
  ser->writeObject(*laser_message);
  std::cerr << "LaserMessage Serialized" << std::endl;

  delete laser_message;

  timestamp = getTime();
  ++seq;

  std::vector<float> range_sensor_ranges(range_size);
  for (int i = 0; i < range_size; ++i) {
    range_sensor_ranges[i] = range_size * 0.05;
  }

  RangeMessage* range_message = new RangeMessage("/range", "/range", seq,
      timestamp);
  range_message->ranges.setValue(range_sensor_ranges);
  range_message->min_range.setValue(0);
  range_message->max_range.setValue(1.5);
  range_message->field_of_view.setValue(2.0);
  range_message->radiation_type.setValue(ULTRASOUND);

  std::cerr << "Serializing RangeMessage" << std::endl;
  ser->writeObject(*range_message);
  std::cerr << "RangeMessage Serialized" << std::endl;

  delete range_message;

  timestamp = getTime();
  ++seq;

  IMUMessage* imu_message = new IMUMessage("/imu", "/imu", seq, timestamp);
  imu_message->angular_velocity.setValue(Vector3f(1.0, 1.2, 1.4));
  imu_message->linear_acceleration.setValue(Vector3f(2.0, 2.2, 2.4));
  imu_message->orientation.setValue(Quaternionf(1, 2, 3, 4).normalized());

  std::cerr << "Serializing IMUMessage" << std::endl;
  ser->writeObject(*imu_message);
  std::cerr << "IMUMessage Serialized" << std::endl;

  delete imu_message;

  timestamp = getTime();
  ++seq;

  JointStateMessage* joint_message = new JointStateMessage("/joint", "/joint",
      seq, timestamp);
  joint_message->joint_states.resize(2);

  JointStatus& joint_1 = joint_message->joint_states.value(0);
  joint_1.name = "joint1";
  joint_1.effort = 0.25;
  joint_1.position = 237;
  joint_1.velocity = 1.36;

  JointStatus& joint_2 = joint_message->joint_states.value(1);
  joint_2.name = "joint2";
  joint_2.effort = 0.253;
  joint_2.position = 23741;
  joint_2.velocity = 1.3612;

  std::cerr << "Serializing JointStateMessage" << std::endl;
  ser->writeObject(*joint_message);
  std::cerr << "JointStateMessage Serialized" << std::endl;

  delete joint_message;

  timestamp = getTime();
  ++seq;

  PoseMessage* pose_message = new PoseMessage("/pose", "/pose", seq, timestamp);

  Isometry3f p;
  p.linear() = geometry3d::a2r(Vector3f(0, -0.707107, 0));
  p.translation() = Vector3f(1, -20, 10);
  pose_message->pose.setValue(p);

  std::cerr << "Serializing pose_message_ea_p" << std::endl;
  ser->writeObject(*pose_message);
  std::cerr << "pose_message_ea_p Serialized" << std::endl;

  delete pose_message;

  timestamp = getTime();
  ++seq;

  int rows = 200;
  int cols = 100;

  const float step = 1 / (float) (rows * cols);
  ImageUInt8* uint8 = new ImageUInt8(rows, cols);
  const uint8_t max_uint8 = 255;
  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const uint8_t scaled_uint8 = value * max_uint8;
      uint8->at(r, c) = scaled_uint8;
      value += step;
    }
  }
  ImageUInt8* uint8_2 = new ImageUInt8(*uint8);

  ImageMessage* image_message = new ImageMessage("/pinhole_image",
      "/pinhole_image", seq, timestamp);

  image_message->depth_scale.setValue(0.123);
  Matrix3f camera_matrix;
  camera_matrix << 1.0, 0, 720, 0, 1.0, 380, 0, 0, 1;
  image_message->projection_model.setValue("pinhole");
  image_message->camera_matrix.setValue(camera_matrix);
  Eigen::VectorXd dcoeff(2);
  dcoeff.fill(1.0);
  image_message->distortion_coefficients.setValue(dcoeff);

  ImageData* idata = new ImageData;
  image_message->setImage(uint8);

  std::cerr << "Serializing image_message" << std::endl;
  ser->writeObject(*image_message);
  std::cerr << "image_message Serialized" << std::endl;

  delete image_message;

  timestamp = getTime();
  ++seq;

  ImageMessage* spherical_image_message = new ImageMessage("/spherical_image",
      "/spherical_image", seq, timestamp);

  //spherical_image_message->setNameAttribute("my_spherical_image");
  spherical_image_message->depth_scale.setValue(0.234);
  spherical_image_message->projection_model.setValue("spherical");
  spherical_image_message->setCameraMatrixFromSphericalImageParameters(
      Vector4f(1, 2, 3, 4));
  spherical_image_message->distortion_coefficients.setValue(dcoeff);
  spherical_image_message->setImage(uint8_2);

  std::cerr << "Serializing spherical_image_message" << std::endl;
  ser->writeObject(*spherical_image_message);
  std::cerr << "spherical_image_message Serialized" << std::endl;

  delete spherical_image_message;

  timestamp = getTime();
  ++seq;

  StaticTransformTreeMessage* tree_message = new StaticTransformTreeMessage(
      "/static_tree", "/static_tree", seq, timestamp);
  StaticTransformTree& tree = tree_message->tree.value();
  StaticTransform t0("/f0", "/f1", Isometry3f::Identity());
  StaticTransform t1("/f1", "/f2", Isometry3f::Identity());
  StaticTransform t2("/f2", "/f3", Isometry3f::Identity());
  StaticTransform t3("/f1", "/f4", Isometry3f::Identity());
  Isometry3f iso;
  iso.setIdentity();
  iso.translation() = Vector3f::Ones();
  StaticTransform t4("/f1", "/f2", iso);
  std::cerr << "is well formed?" << std::endl;
  tree.isWellFormed(true);

  tree.addStaticTransform(t0);
  tree.addStaticTransform(t1);
  tree.addStaticTransform(t2);
  tree.addStaticTransform(t3);
  std::cerr << "is well formed?" << std::endl;
  tree.isWellFormed(true);
  std::cerr << std::endl;
  std::cerr << std::endl;
  std::cerr << std::endl;

  std::cerr << "update Transform" << std::endl;
  if (!tree.updateTransform(t4)) {
    std::cerr << "unable to update" << std::endl;
  }
  std::cerr << std::endl;

  std::cerr << "Serializing tree_message" << std::endl;
  ser->writeObject(*tree_message);
  std::cerr << "tree_message Serialized" << std::endl;

  delete tree_message;
  delete ser;
  delete context;
  /**/

  std::cerr << "starting deserialization" << std::endl;
  Deserializer* deser = new Deserializer();
  deser->setFilePath(argv[1]);
  Serializable* object;
  while (object = deser->readObject()) {
    std::cerr << "o: " << object << std::endl;
    std::cerr << "type: " << object->className() << std::endl;
    delete object;
  }

  delete deser;

  /**/
  return 0;

}
