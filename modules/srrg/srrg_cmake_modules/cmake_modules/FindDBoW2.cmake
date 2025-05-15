# Find the header files
set(DBoW2_DIR $ENV{HOME}/source/libraries/DBoW2)
message("srrg_cmake_modules|searching for DBoW2 in: " ${DBoW2_DIR})

FIND_PATH(DBoW2_INCLUDE_DIR
  NAMES DBoW2/DBoW2.h
  HINTS ${DBoW2_DIR} )



if(DBoW2_INCLUDE_DIR)
  message("srrg_cmake_modules|found DBoW2:  ${DBoW2_INCLUDE_DIR}")
  find_library(DBoW2_LIBRARY DBoW2
    HINTS ${DBoW2_INCLUDE_DIR}/../lib)
  if(DBoW2_LIBRARY)
    message("srrg_cmake_modules|DBoW2 LIB:  ${DBoW2_LIBRARY}")
  else()
    set(DBoW2_LIBRARY ${DBoW2_INCLUDE_DIR}/../lib/libDBoW2.so)
    message("srrg_cmake_modules|DBoW2 LIB not found | bdc HARDCODING")
    message("srrg_cmake_modules|DBoW2 LIB:  ${DBoW2_LIBRARY}")
  endif()
else()
  message("srrg_cmake_modules|DBoW2 not found | bdc HARDCODING")
  set(DBoW2_INCLUDE_DIR ${DBoW2_DIR}/include)
  set(DBoW2_LIBRARY ${DBoW2_INCLUDE_DIR}/../build/libDBoW2.so)
  message("srrg_cmake_modules|DBoW2 INCLUDE:  ${DBoW2_INCLUDE_DIR}")
  message("srrg_cmake_modules|DBoW2 LIB    :  ${DBoW2_LIBRARY}")
endif()

set(DBoW2_DIRS ${DBoW2_DIR})
set(DBoW2_INCLUDE_DIRS ${DBoW2_INCLUDE_DIR})
set(DBoW2_LIBRARIES ${DBoW2_LIBRARY})
