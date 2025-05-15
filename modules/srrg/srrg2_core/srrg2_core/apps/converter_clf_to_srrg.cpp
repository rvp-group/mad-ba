#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts AIS formatted CLF files to SRRG format", 0};
// clang-format on

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_input_file (
    &command_line_parser, "i",  "input", "file to convert", "");
  ArgumentString argument_output_file (
    &command_line_parser, "o",  "output", "converted output file", "out.json");
  ArgumentString argument_laser_topic (
     &command_line_parser, "lt",  "laser-topic", "laser topic name (without /)", "FLASER");
  ArgumentString argument_odometry_topic (
     &command_line_parser, "ot",  "odometry-topic", "odometry topic name (without /)", "ODOM");

  // clang-format on
  command_line_parser.parse();
  if (!argument_input_file.isSet() || argument_input_file.value().empty()) {
    std::cerr << "ERROR: input file not set" << std::endl;
    return 0;
  }
  if (!argument_output_file.isSet() || argument_output_file.value().empty()) {
    std::cerr << "ERROR: output file not set" << std::endl;
    return 0;
  }

  // ds configure serializer
  Serializer serializer;
  serializer.setFilePath(argument_output_file.value());

  // ds open file
  std::ifstream input_file(argument_input_file.value());
  if (!input_file.good() || !input_file.is_open()) {
    std::cerr << "ERROR: unable to open input file" << std::endl;
  }

  // ds parse file line by line
  std::string line_buffer;
  size_t number_of_converted_laser_messages    = 0;
  size_t number_of_converted_odometry_messages = 0;
  while (std::getline(input_file, line_buffer)) {
    // ds check if prefix matches the laser topic
    if (line_buffer.find(argument_laser_topic.value()) == 0) {
      // clang-format off
      // ds format: num_readings [range_readings] x y theta odom_x odom_y odom_theta ipc_timestamp ipc_hostname logger_timestamp
      // clang-format on

      // ds stream tokens - first we have to extract the number of readings
      std::istringstream line_buffer_stream(line_buffer);
      std::string topic_name;
      size_t num_readings;
      line_buffer_stream >> topic_name >> num_readings;
      std::vector<float> range_readings;
      range_readings.reserve(num_readings);
      for (size_t i = 0; i < num_readings; ++i) {
        double range_reading = 0;
        line_buffer_stream >> range_reading;
        range_readings.emplace_back(range_reading);
      }
      double x, y, theta, odom_x, odom_y, odom_theta, ipc_timestamp;
      line_buffer_stream >> x >> y >> theta >> odom_x >> odom_y >> odom_theta >>
        ipc_timestamp;

      // ds assemble a laser message TODO fetch laser calibration
      LaserMessage message("/" + topic_name,
                           "base_link",
                           number_of_converted_laser_messages,
                           ipc_timestamp);
      message.angle_increment.setValue(0.0174533 /*killian*/);
      message.angle_max.setValue(1.55334 /*killian*/);
      message.angle_min.setValue(-1.5708 /*killian*/);
      message.intensities.value().clear();
      message.range_max.setValue(20 /*killian*/);
      message.range_min.setValue(0 /*killian*/);
      message.ranges.setValue(range_readings);

      // ds write messge
      serializer.writeObject(message);
      ++number_of_converted_laser_messages;
      std::cerr << "L";
    }

    // ds otherwise check if it matches the odom topic
    else if (line_buffer.find(argument_odometry_topic.value()) == 0) {
      // clang-format off
      // ds format: ODOM x y theta tv rv accel ipc_timestamp ipc_hostname logger_timestamp
      // clang-format on

      // ds stream tokens
      std::string topic_name;
      double x, y, theta, tv, rv, accel, ipc_timestamp;
      std::istringstream(line_buffer) >> topic_name >> x >> y >> theta >> tv >>
        rv >> accel >> ipc_timestamp;

      // ds assemble an odometry message
      OdometryMessage message("/" + topic_name,
                              "odom",
                              number_of_converted_odometry_messages,
                              ipc_timestamp);
      message.angular_velocity.setValue(Vector3f(0, 0, rv));
      message.child_frame.setValue("base_link");
      message.linear_velocity.setValue(Vector3f(tv, 0, 0));
      Isometry3f& pose(message.pose.value());
      pose.setIdentity();
      pose.translation().x() = x;
      pose.translation().y() = y;
      pose.rotate(AngleAxisf(theta, Vector3f::UnitZ()));

      // ds write messge
      serializer.writeObject(message);
      ++number_of_converted_odometry_messages;
      std::cerr << "O";
    } else {
      std::cerr << "-";
    }
  }
  std::cerr << std::endl;
  input_file.close();

  std::cerr << "# converted laser messages: "
            << number_of_converted_laser_messages << std::endl;
  std::cerr << "# converted odometry messages: "
            << number_of_converted_odometry_messages << std::endl;
  return 0;
}
