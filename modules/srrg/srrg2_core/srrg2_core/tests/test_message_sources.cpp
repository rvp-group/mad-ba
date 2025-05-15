#include "srrg_messages/instances.h"
#include <srrg_data_structures/platform.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_property/property_container.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/messages/point_stamped_message.h>
#include <srrg_messages/messages/twist_stamped_message.h>
#include <srrg_messages/messages/navsat_fix_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_messages/messages/pose_with_covariance_stamped_message.h>
#include <srrg_messages/messages/pose_array_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/cmd_vel_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

// ds test helper function
void generateMessagesOnDisk(const std::string& file_name_,
                            const size_t number_of_messages_,
                            const std::vector<std::string> topics_,
                            const std::vector<double>& time_offsets_,
                            bool alternate_topic_order = false) {
  Serializer serializer;
  serializer.setFilePath(file_name_);
  for (size_t index_message = 0; index_message < number_of_messages_; ++index_message) {
    const double timestamp_seconds = index_message / 10.0;
    if (alternate_topic_order && index_message % 2) {
      for (int64_t index_topic = topics_.size() - 1; index_topic >= 0; --index_topic) {
        const std::string& topic = topics_[index_topic];
        PointStampedMessagePtr message(new PointStampedMessage(
          topic, topic, index_message, timestamp_seconds + time_offsets_[index_topic]));
        serializer.writeObject(*message);
      }
    } else {
      for (size_t index_topic = 0; index_topic < topics_.size(); ++index_topic) {
        const std::string& topic = topics_[index_topic];
        PointStampedMessagePtr message(new PointStampedMessage(
          topic, topic, index_message, timestamp_seconds + time_offsets_[index_topic]));
        serializer.writeObject(*message);
      }
    }
  }
}

int main(int argc_, char** argv_) {
  messages_registerTypes();
  return srrg2_test::runTests(argc_, argv_);
}

TEST(MessageSynchronizedSource, ConstructionAndDestruction) {
  MessageFileSourcePtr source(new MessageFileSource());
  MessageSortedSourcePtr sorter(new MessageSortedSource());
  MessageSynchronizedSourcePtr synchronized_source(new MessageSynchronizedSource());
}

TEST(MessageSynchronizedSource, ValidTopics_NoTimeOffset) {
  const std::string message_file_name    = "test_messages.json";
  constexpr size_t number_of_messages    = 10;
  const std::vector<std::string> topics  = {"/topic0", "/topic1"};
  const std::vector<double> time_offsets = {0, 0};
  generateMessagesOnDisk(message_file_name, number_of_messages, topics, time_offsets);

  // ds configure a synchronized message source
  MessageFileSourcePtr source(new MessageFileSource());
  MessageSortedSourcePtr sorter(new MessageSortedSource());
  sorter->param_source.setValue(source);
  sorter->param_time_interval.setValue(1e-5);
  MessageSynchronizedSourcePtr synchronized_source(new MessageSynchronizedSource());
  synchronized_source->param_source.setValue(sorter);
  synchronized_source->param_topics.value().push_back(topics[0]);
  synchronized_source->param_topics.value().push_back(topics[1]);
  synchronized_source->param_time_interval.setValue(1e-5);

  // ds open a synthetic message file
  source->open(message_file_name);
  ASSERT_TRUE(source->isOpen());
  ASSERT_EQ(source->param_filename.value(), message_file_name);

  // ds process messages
  size_t number_of_processed_message_packs = 0;
  while (BaseSensorMessagePtr message = synchronized_source->getMessage()) {
    // ASSERT_TRUE(synchronized_source->isOpen()); //ds TODO why is this FALSE?
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    ASSERT_NOTNULL(message_pack);
    ++number_of_processed_message_packs;
  }
  ASSERT_EQ(number_of_processed_message_packs, number_of_messages);
  ASSERT_EQ(synchronized_source->numDroppedMessages(), 0);

  source->close();
  ASSERT_FALSE(source->isOpen());
  ASSERT_FALSE(synchronized_source->isOpen());
}

TEST(MessageSynchronizedSource, InvalidTopics_NoTimeOffset) {
  const std::string message_file_name    = "test_messages.json";
  constexpr size_t number_of_messages    = 10;
  const std::vector<std::string> topics  = {"/topic0", "/topic1"};
  const std::vector<double> time_offsets = {0, 0};
  generateMessagesOnDisk(message_file_name, number_of_messages, topics, time_offsets);

  // ds configure a synchronized message source
  MessageFileSourcePtr source(new MessageFileSource());
  MessageSortedSourcePtr sorter(new MessageSortedSource());
  sorter->param_source.setValue(source);
  sorter->param_time_interval.setValue(1e-5);
  MessageSynchronizedSourcePtr synchronized_source(new MessageSynchronizedSource());
  synchronized_source->param_source.setValue(sorter);
  synchronized_source->param_topics.value().push_back("/mirco");
  synchronized_source->param_topics.value().push_back("/micro");
  synchronized_source->param_time_interval.setValue(1e-5);

  // ds open a synthetic message file
  source->open(message_file_name);
  ASSERT_TRUE(source->isOpen());
  ASSERT_EQ(source->param_filename.value(), message_file_name);

  // ds process messages
  size_t number_of_processed_message_packs = 0;
  while (BaseSensorMessagePtr message = synchronized_source->getMessage()) {
    // ASSERT_TRUE(synchronized_source->isOpen()); //ds TODO why is this FALSE?
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    ASSERT_NOTNULL(message_pack);
    ++number_of_processed_message_packs;
  }
  ASSERT_EQ(number_of_processed_message_packs, static_cast<size_t>(0));
  ASSERT_EQ(synchronized_source->numDroppedMessages(), 0); // ds TODO no drops intended?

  source->close();
  ASSERT_FALSE(source->isOpen());
  ASSERT_FALSE(synchronized_source->isOpen());
}

TEST(MessageSynchronizedSource, ValidTopics_TolerableTimeOffset) {
  const std::string message_file_name    = "test_messages.json";
  constexpr size_t number_of_messages    = 10;
  const std::vector<std::string> topics  = {"/topic0", "/topic1"};
  const std::vector<double> time_offsets = {-1e-6, 1e-6};
  generateMessagesOnDisk(message_file_name, number_of_messages, topics, time_offsets);

  // ds configure a synchronized message source
  MessageFileSourcePtr source(new MessageFileSource());
  MessageSortedSourcePtr sorter(new MessageSortedSource());
  sorter->param_source.setValue(source);
  sorter->param_time_interval.setValue(1e-5);
  MessageSynchronizedSourcePtr synchronized_source(new MessageSynchronizedSource());
  synchronized_source->param_source.setValue(sorter);
  synchronized_source->param_topics.value().push_back(topics[0]);
  synchronized_source->param_topics.value().push_back(topics[1]);
  synchronized_source->param_time_interval.setValue(1e-5);

  // ds open a synthetic message file
  source->open(message_file_name);
  ASSERT_TRUE(source->isOpen());
  ASSERT_EQ(source->param_filename.value(), message_file_name);

  // ds process messages
  size_t number_of_processed_message_packs = 0;
  while (BaseSensorMessagePtr message = synchronized_source->getMessage()) {
    // ASSERT_TRUE(synchronized_source->isOpen()); //ds TODO why is this FALSE?
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    ASSERT_NOTNULL(message_pack);
    ++number_of_processed_message_packs;
  }
  ASSERT_EQ(number_of_processed_message_packs, number_of_messages);
  ASSERT_EQ(synchronized_source->numDroppedMessages(), 0);

  source->close();
  ASSERT_FALSE(source->isOpen());
  ASSERT_FALSE(synchronized_source->isOpen());
}

TEST(MessageSynchronizedSource, ValidTopics_IntolerableTimeOffset) {
  const std::string message_file_name    = "test_messages.json";
  constexpr size_t number_of_messages    = 10;
  const std::vector<std::string> topics  = {"/topic0", "/topic1"};
  const std::vector<double> time_offsets = {-0.01, 0.01};
  generateMessagesOnDisk(message_file_name, number_of_messages, topics, time_offsets);

  // ds configure a synchronized message source
  MessageFileSourcePtr source(new MessageFileSource());
  MessageSortedSourcePtr sorter(new MessageSortedSource());
  sorter->param_source.setValue(source);
  sorter->param_time_interval.setValue(1e-5);
  MessageSynchronizedSourcePtr synchronized_source(new MessageSynchronizedSource());
  synchronized_source->param_source.setValue(sorter);
  synchronized_source->param_topics.value().push_back(topics[0]);
  synchronized_source->param_topics.value().push_back(topics[1]);
  synchronized_source->param_time_interval.setValue(1e-5);

  // ds open a synthetic message file
  source->open(message_file_name);
  ASSERT_TRUE(source->isOpen());
  ASSERT_EQ(source->param_filename.value(), message_file_name);

  // ds process messages
  size_t number_of_processed_message_packs = 0;
  while (BaseSensorMessagePtr message = synchronized_source->getMessage()) {
    // ASSERT_TRUE(synchronized_source->isOpen()); //ds TODO why is this FALSE?
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    ASSERT_NOTNULL(message_pack);
    ++number_of_processed_message_packs;
  }
  ASSERT_EQ(number_of_processed_message_packs, size_t(0));
  // ds TODO why not 20 dropped messages?
  ASSERT_EQ(static_cast<size_t>(synchronized_source->numDroppedMessages()),
            2 * number_of_messages - 2);

  source->close();
  ASSERT_FALSE(source->isOpen());
  ASSERT_FALSE(synchronized_source->isOpen());
}

TEST(MessageSynchronizedSource, ValidTopics_ShuffledTimeOffset) {
  const std::string message_file_name    = "test_messages.json";
  constexpr size_t number_of_messages    = 10;
  const std::vector<std::string> topics  = {"/topic0", "/topic1", "/topic2"};
  const std::vector<double> time_offsets = {123456789.01, 123456789.02, 123456789.03};
  generateMessagesOnDisk(message_file_name, number_of_messages, topics, time_offsets, true);

  // ds configure a synchronized message source
  MessageFileSourcePtr source(new MessageFileSource());
  MessageSortedSourcePtr sorter(new MessageSortedSource());
  sorter->param_source.setValue(source);
  sorter->param_time_interval.setValue(0.05);
  MessageSynchronizedSourcePtr synchronized_source(new MessageSynchronizedSource());
  synchronized_source->param_source.setValue(sorter);
  synchronized_source->param_topics.value().push_back(topics[0]);
  synchronized_source->param_topics.value().push_back(topics[1]);
  synchronized_source->param_topics.value().push_back(topics[2]);
  synchronized_source->param_time_interval.setValue(0.05);

  // ds open a synthetic message file
  source->open(message_file_name);
  ASSERT_TRUE(source->isOpen());
  ASSERT_EQ(source->param_filename.value(), message_file_name);

  // ds process messages
  size_t number_of_processed_message_packs = 0;
  while (BaseSensorMessagePtr message = synchronized_source->getMessage()) {
    // ASSERT_TRUE(synchronized_source->isOpen()); //ds TODO why is this FALSE?
    MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
    ASSERT_NOTNULL(message_pack);
    ++number_of_processed_message_packs;
  }
  ASSERT_EQ(number_of_processed_message_packs, number_of_messages);
  ASSERT_EQ(synchronized_source->numDroppedMessages(), 0);

  source->close();
  ASSERT_FALSE(source->isOpen());
  ASSERT_FALSE(synchronized_source->isOpen());
}
