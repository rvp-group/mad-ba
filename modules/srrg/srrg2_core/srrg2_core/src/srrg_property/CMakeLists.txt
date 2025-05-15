add_library(srrg2_property_library SHARED

  property.h
  property.cpp

  property_eigen.h
  property_eigen.cpp
  
  property_identifiable.cpp
  property_identifiable.h
  
  property_container.h
  property_container.cpp

  property_container_manager.cpp
  property_container_manager.h
  
  )
target_link_libraries(srrg2_property_library
  srrg2_boss_library
  ${CMAKE_DL_LIBS})
