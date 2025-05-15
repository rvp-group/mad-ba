add_library(srrg2_messages_library SHARED
  messages/base_sensor_message.cpp messages/base_sensor_message.h
  messages/camera_info_message.cpp messages/camera_info_message.h
  messages/image_message.cpp messages/image_message.h
  messages/imu_message.cpp messages/imu_message.h
  messages/range_message.cpp messages/range_message.h
  messages/laser_message.cpp messages/laser_message.h
  messages/odometry_message.cpp messages/odometry_message.h
  messages/point_stamped_message.cpp messages/point_stamped_message.h
  messages/ticks_message.cpp messages/ticks_message.h
  messages/transform_events_message.cpp messages/transform_events_message.h
  messages/twist_stamped_message.cpp messages/twist_stamped_message.h
  messages/point_field.cpp messages/point_field.h 
  messages/point_cloud2_message.cpp messages/point_cloud2_message.h
  messages/navsat_fix_message.h messages/navsat_fix_message.cpp
  messages/pose_message.h messages/pose_message.cpp
  messages/pose_array_message.h messages/pose_array_message.cpp
  messages/pose_stamped_message.h messages/pose_stamped_message.cpp
  messages/pose_with_covariance_stamped_message.h messages/pose_with_covariance_stamped_message.cpp
  messages/path_message.h messages/path_message.cpp
  messages/cmd_vel_message.h messages/cmd_vel_message.cpp
  messages/grid_map_message.h messages/grid_map_message.cpp
  messages/planner_status_message.h messages/planner_status_message.cpp

  message_handlers/message_pack.cpp message_handlers/message_pack.h
  message_handlers/message_sink_base.cpp message_handlers/message_sink_base.h
  message_handlers/message_source_base.cpp message_handlers/message_source_base.h
  message_handlers/message_filter_base.cpp message_handlers/message_filter_base.h

  message_handlers/message_selector_sink.cpp message_handlers/message_selector_sink.h
  message_handlers/message_file_sink.cpp message_handlers/message_file_sink.h
  message_handlers/message_file_sink_base.cpp message_handlers/message_file_sink_base.h

  message_handlers/message_file_source_base.cpp message_handlers/message_file_source_base.h
  message_handlers/message_file_source.cpp message_handlers/message_file_source.h
  message_handlers/message_sorted_source.cpp message_handlers/message_sorted_source.h
  message_handlers/message_synchronized_source.cpp message_handlers/message_synchronized_source.h
  message_handlers/message_source_platform.cpp message_handlers/message_source_platform.h
  message_handlers/message_odom_subsampler_source.cpp message_handlers/message_odom_subsampler_source.h
  message_handlers/message_sorted_sink.cpp message_handlers/message_sorted_sink.h
  message_handlers/message_synchronized_sink.cpp message_handlers/message_synchronized_sink.h
  message_handlers/message_odom_subsampler_sink.cpp message_handlers/message_odom_subsampler_sink.h
  message_handlers/message_platform_listener_sink.cpp message_handlers/message_platform_listener_sink.h
  message_handlers/image_message_viewer.cpp message_handlers/image_message_viewer.h
  instances.cpp instances.h
  messages/joints_message.cpp messages/joints_message.h)

target_link_libraries(srrg2_messages_library
  srrg2_property_library
  srrg2_config_library
  srrg2_viewer_library
  srrg2_image_library
  srrg2_data_structures_library
  srrg2_system_utils_library
)
