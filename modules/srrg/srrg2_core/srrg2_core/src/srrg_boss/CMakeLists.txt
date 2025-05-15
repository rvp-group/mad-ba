add_library(srrg2_boss_library SHARED
  blob.cpp blob.h     
  deserializer.cpp deserializer.h
  id_context.cpp id_context.h   
  identifiable.cpp identifiable.h
  id_placeholder.cpp id_placeholder.h
  json_recursive_object_parser.cpp json_recursive_object_parser.h 
  json_object_writer.cpp json_object_writer.h
  object_data.cpp object_data.h
  object_parser.cpp object_parser.h
  object_writer.cpp object_writer.h
  serializable.cpp serializable.h  
  serialization_context.cpp serialization_context.h
  serializer.cpp serializer.h
  stream.cpp stream.h
)
