#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/instances.h>
#include <srrg_boss/serializer.h>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::message_sorter_example| "

using namespace srrg2_core;

int main(int argc, char** argv) {
  messages_registerTypes();

  EXAMPLE_LOG << "Generating un-sorted messages" << std::endl;
  IMUMessage msg0("/imu0", "/imu", 0, 0.1);
  IMUMessage msg1("/imu0", "/imu", 1, 0.2);
  IMUMessage msg2("/imu0", "/imu", 2, 0.5);
  IMUMessage msg3("/imu0", "/imu", 3, 0.3);
  IMUMessage msg4("/imu0", "/imu", 4, 0.2);
  IMUMessage msg5("/imu0", "/imu", 5, 0.5);
  IMUMessage msg6("/imu0", "/imu", 6, 0.4);
  IMUMessage msg7("/imu0", "/imu", 7, 0.6);
  IMUMessage msg8("/imu0", "/imu", 8, 0.55);
  IMUMessage msg9("/imu0", "/imu", 9, 0.45);

  const std::string json_file = "sorter_test.json";
  Serializer ser;
  EXAMPLE_LOG << "Serializing messages on " << json_file << std::endl;
  ser.setFilePath(json_file);
  ser.writeObject(msg0);
  ser.writeObject(msg1);
  ser.writeObject(msg2);
  ser.writeObject(msg3);
  ser.writeObject(msg4);
  ser.writeObject(msg5);
  ser.writeObject(msg6);
  ser.writeObject(msg7);
  ser.writeObject(msg8);
  ser.writeObject(msg9);

  EXAMPLE_LOG << "Sourcing " << json_file << std::endl;
  MessageFileSourcePtr src(new MessageFileSource);
  EXAMPLE_LOG << "Sourced messages:" << std::endl;
  BaseSensorMessagePtr msg;
  src->open(json_file);
  while ( (msg = src->getMessage())  ) {
    EXAMPLE_LOG << "Ts: " << msg->timestamp.value() << "\t Seq: " << msg->seq.value() << std::endl;
  }

  EXAMPLE_LOG << "Sorting " << json_file << std::endl;
  MessageSortedSourcePtr sorter(new MessageSortedSource);
  src->open(json_file);
  sorter->param_source.setValue(src);
  sorter->param_time_interval.setValue(0.3);

  EXAMPLE_LOG << "Sorted messages:" << std::endl;
  while ( (msg = sorter->getMessage())  ) {
    EXAMPLE_LOG << "Ts: " << msg->timestamp.value() << "\t Seq: " << msg->seq.value() << std::endl;
  }

  return EXIT_SUCCESS;
}
