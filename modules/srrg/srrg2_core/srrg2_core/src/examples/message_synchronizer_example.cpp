#include <unistd.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_boss/serializer.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/instances.h>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::message_synchronizer_example| "

using namespace srrg2_core;
using namespace std;

int main(int argc, char** argv) {
  messages_registerTypes();

  const std::string json_file = "sync_test.json";
  const int max_seq = 20;
  
  EXAMPLE_LOG << "Generating an un-synch log w/ IMU and Images" << std::endl;
  EXAMPLE_LOG << max_seq << " messages stored in " << json_file << std::endl;
  Serializer ser;
  ser.setFilePath(json_file);
  for (int i = 0; i < max_seq; ++i) {
    double image_t = getTime();
    ImageMessage* img_msg = new ImageMessage("/img0", "/img0", i, image_t);
    ImageUInt8* img = new ImageUInt8(100, 200);
    for (size_t r = 0; r < img->rows(); ++r) {
      for (size_t c = 0; c < img->cols(); ++c) {
        img->at(r, c) = (r + c) % 256;
      }
    }
    img_msg->setImage(img);
    ser.writeObject(*img_msg);
    delete img_msg;
    usleep(1000);
    double imu_t = getTime();
    IMUMessage* imu_msg = new IMUMessage("/imu", "/imu", i + 10, imu_t);
    ser.writeObject(*imu_msg);
    delete imu_msg;
    EXAMPLE_LOG << std::setprecision(20) << "Image time: " << image_t << "\tImu time: " << imu_t << std::endl;
  }

  EXAMPLE_LOG << "Sourcing messages from " << json_file << std::endl;
  MessageFileSourcePtr src(new MessageFileSource);
  src->open(json_file);

  EXAMPLE_LOG << "Sorting messages" << std::endl;
  MessageSortedSourcePtr sorter(new MessageSortedSource);
  sorter->param_source.setValue(src);
  sorter->param_time_interval.setValue(0.3);

  EXAMPLE_LOG << "Synch-ing messages" << std::endl;
  MessageSynchronizedSourcePtr sync(new MessageSynchronizedSource);
  sync->param_source.setValue(sorter);
  sync->param_topics.value().push_back("/img0");
  sync->param_topics.value().push_back("/imu");
  sync->param_time_interval.setValue(0.001);

  BaseSensorMessagePtr msg;
  while ((msg = sync->getMessage())) {
    std::shared_ptr<MessagePack> pack = dynamic_pointer_cast<MessagePack>(msg);
    if (pack) {
      EXAMPLE_LOG << "Image time: " << (*pack->property<BaseSensorMessagePtr>("/img0"))->timestamp.value()
                  << "\tImu time: " << (*pack->property<BaseSensorMessagePtr>("/imu"))->timestamp.value()
                  << std::endl;
    }
  }

}
