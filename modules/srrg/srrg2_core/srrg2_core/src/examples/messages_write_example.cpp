#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_boss/id_context.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/instances.h>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::message_write_example| "

using namespace srrg2_core;
using namespace std;

const char* banner[]={
    "srrg_message_write_example",
    "usage: srrg_message_write_example <file>",
    0
};

int main(int argc, char** argv) {
  messages_registerTypes();

  const size_t laser_size = 10;
  const size_t range_size = 1;

  if (argc == 2) {
    EXAMPLE_LOG << "Write file to " << argv[1] << endl;
  } else {
    printBanner(banner);
    return 0;
  }


  SerializationContext* context = new SerializationContext();
  Serializer* ser = new Serializer(context);
  ser->serializationContext()->setOutputFilePath(argv[1]);

  EXAMPLE_LOG << "Creating and Serializing a " <<  FG_GREEN("LaserMessage") << endl;
  vector<float> intensities(laser_size);
  vector<float> ranges(laser_size);
  float value = 0;
  for (size_t i = 0; i < laser_size; ++i) {
    intensities[i] = value * 50;
    ranges[i] = value * 100;
    value += 1/static_cast<float>(laser_size);
  }
  int seq = 10;
  double timestamp = getTime();
 
  LaserMessage* laser_message = new LaserMessage("/scan", "/scan", seq, timestamp);
  Isometry3f offset;
  Matrix4f tmp;
  tmp << 0, 1, 0, 3, -1, 0, 0, 2, 0, 0, 1, 1, 0, 0, 0, 1;
  offset.linear() = tmp.block<3,3>(0,0);
  offset.translation() = tmp.block<3,1>(0,3);
  laser_message->angle_increment.setValue(0.3);
  laser_message->intensities.setValue(intensities);
  laser_message->range_min.setValue(0);
  laser_message->angle_min.setValue(0);
  laser_message->angle_max.setValue(100);
  laser_message->range_max.setValue(100);
  laser_message->ranges.setValue(ranges);
  laser_message->scan_time.setValue(getTime());
  laser_message->time_increment.setValue(0.001);
  
  EXAMPLE_LOG << "Serializing LaserMessage" << endl;
  ser->writeObject(*laser_message);
  delete laser_message;
  EXAMPLE_LOG << "LaserMessage Serialized" << endl << endl;


  EXAMPLE_LOG << "Creating and Serializing a " << FG_GREEN("RangeMessage") << endl;
  timestamp = getTime();
  ++seq;
  float range_sensor_range(range_size * 0.05);

  RangeMessage* range_message = new RangeMessage("/range", "/range", seq, timestamp);
  range_message->range.setValue(range_sensor_range);
  range_message->range_min.setValue(0);
  range_message->range_max.setValue(1.5);
  range_message->field_of_view.setValue(2.0);
  range_message->radiation_type.setValue(ULTRASOUND);

  EXAMPLE_LOG << "Serializing RangeMessage" << endl;
  ser->writeObject(*range_message);
  delete range_message;
  EXAMPLE_LOG << "RangeMessage Serialized" << endl << endl;

  
  EXAMPLE_LOG << "Creating and Serializing a " << FG_GREEN("IMUMessage") << endl;
  timestamp = getTime();
  ++seq;
  IMUMessage* imu_message = new IMUMessage("/imu", "/imu", seq, timestamp);
  imu_message->angular_velocity.setValue(Vector3f(1.0,1.2,1.4));
  imu_message->linear_acceleration.setValue(Vector3f(2.0,2.2,2.4));
  imu_message->orientation.setValue(Quaternionf(1,2,3,4).normalized().matrix());

  EXAMPLE_LOG << "Serializing IMUMessage" << endl;
  ser->writeObject(*imu_message);
  delete imu_message;
  EXAMPLE_LOG << "IMUMessage Serialized" << endl << endl;


  EXAMPLE_LOG << "Creating and Serializing a " << FG_GREEN("JointStateMessage") << endl;
  timestamp = getTime();
  ++seq;
  JointsMessage* joint_message = new JointsMessage("/joint", "/joint", seq, timestamp);
  joint_message->joint_events.resize(2);
  const double position_1 = 0;
  const double effort_1   = 0.25;
  const double velocity_1 = 1.36;
  joint_message->joint_events.value(0) = JointEvent(timestamp, "joint1", position_1, effort_1, velocity_1);
  const double position_2 = 1234;
  const double effort_2   = 0.253;
  const double velocity_2 = 1.3612;
  joint_message->joint_events.value(1) = JointEvent(timestamp, "joint2", position_2, effort_2, velocity_2);

  EXAMPLE_LOG << "Serializing JointStateMessage" << endl;
  ser->writeObject(*joint_message);
  delete joint_message;
  EXAMPLE_LOG << "JointStateMessage Serialized" << endl << endl;


#warning "TODO: Check for missing messages to Serialize"
  
  EXAMPLE_LOG << "Creating and Serializing a " << FG_GREEN("ImageMessage") << endl;
  timestamp = getTime();
  ++seq;
  int rows = 200;
  int cols = 100;
  const float step = 1/(float)(rows*cols);
  ImageUInt8* uint8      = new ImageUInt8(rows, cols);
  const uint8_t max_uint8 = 255;
  EXAMPLE_LOG << "Creating Image" << endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const uint8_t scaled_uint8 = value * max_uint8;
      uint8->at(r,c) = scaled_uint8;
      value += step;
    }
  }
  ImageMessage* image_message = new ImageMessage("/pinhole_image", "/pinhole_image", seq, timestamp);
  image_message->setImage(uint8);

  EXAMPLE_LOG << "Serializing image_message" << endl;
  ser->writeObject(*image_message);
  delete image_message;
  EXAMPLE_LOG << "image_message Serialized" << endl << endl;

  
  EXAMPLE_LOG << "Creating and Serializing a " << FG_GREEN("CameraInfoMessage") << endl;
  timestamp = getTime();
  ++seq;
  Eigen::VectorXd dcoeff(2);
  dcoeff.fill(1.0);
  CameraInfoMessage* camera_info_message = new CameraInfoMessage("/camera_info", "/camera_info", seq, timestamp);
  camera_info_message->depth_scale.setValue(0.234);
  camera_info_message->projection_model.setValue("spherical");
  camera_info_message->distortion_model.setValue("barrel");
  camera_info_message->setCameraMatrixFromSphericalImageParameters(Vector4f(1,2,3,4));
  camera_info_message->distortion_coefficients.setValue(dcoeff);

  EXAMPLE_LOG << "Serializing camera_info_message" << endl;
  ser->writeObject(*camera_info_message);
  delete camera_info_message;
  EXAMPLE_LOG << "camera_info_message Serialized" << endl << endl;



  timestamp = getTime();
  ++seq;


  EXAMPLE_LOG << "Starting Deserialization" << endl;
  Deserializer* deser = new Deserializer();
  deser->setFilePath(argv[1]);
  SerializablePtr object;
  while ( (object = deser->readObjectShared()) ) {
    EXAMPLE_LOG << "o: " << object << endl;
    EXAMPLE_LOG << "type: " << FG_GREEN(object->className()) << endl << endl;
  }

  delete ser;
  delete context;
  delete deser;

  return 0;

}
