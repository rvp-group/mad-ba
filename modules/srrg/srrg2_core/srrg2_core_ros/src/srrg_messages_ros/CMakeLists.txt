add_library(srrg2_messages_ros_library SHARED
  message_handlers/message_ros_source.cpp message_handlers/message_ros_source.cpp
  message_handlers/message_rosbag_source.cpp message_handlers/message_rosbag_source.h
  message_handlers/message_rosbag_sink.cpp message_handlers/message_rosbag_sink.h
  message_handlers/message_ros_sink.cpp message_handlers/message_ros_sink.h

  instances.cpp instances.h

  # message_handlers/message_ros_sink_base.cpp message_handlers/message_ros_sink_base.h
  )

target_link_libraries(srrg2_messages_ros_library
  srrg2_converters_library
)
