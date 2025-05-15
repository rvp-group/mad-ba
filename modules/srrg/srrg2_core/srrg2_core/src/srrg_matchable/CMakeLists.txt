#ia matchable library
add_library(srrg2_matchable_library SHARED
  matchable.h
  matchable.hpp
  descriptor_owner.h
  visual_matchable.h
  instances.h
  instances.cpp)
target_link_libraries(srrg2_matchable_library
  srrg2_boss_library
  srrg2_property_library
  srrg2_point_cloud_library)