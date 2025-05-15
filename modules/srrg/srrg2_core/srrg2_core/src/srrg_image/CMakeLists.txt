add_library(srrg2_image_library SHARED
  image.cpp 			image.h 			
  image_data.cpp 	image_data.h
  )

target_link_libraries(srrg2_image_library
  srrg2_boss_library
  ${OpenCV_LIBS}
  )
