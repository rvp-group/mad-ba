#ia base protocol 
#add_subdirectory(packets)
add_library(srrg2_viewer_core_library SHARED
  packets.h
  color_palette.h
  semaphore.h
  semaphore.cpp
  buffer_memory.h
  buffer_memory.cpp
  buffer_deque.h
  buffer_manager.h
  buffer_manager.cpp
  packets.h
  packet_factory.h
  packet_serializer.h
  packet_serializer.cpp
  packet_deserializer.h
  packet_deserializer.cpp
  packet_processor.h
  buffer_sink.h
  buffer_source.h
  viewport.h
  viewport.cpp
  # shared things
  buffer_sink_shared.h
  buffer_sink_shared.cpp
  buffer_source_shared.h
  buffer_source_shared.cpp
)

target_link_libraries(srrg2_viewer_core_library
  srrg2_property_library
  srrg2_point_cloud_library
  srrg2_matchable_library
  pthread)





