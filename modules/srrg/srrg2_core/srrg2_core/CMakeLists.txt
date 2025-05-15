cmake_minimum_required(VERSION 2.8.3)
project(srrg2_core)

#ds help IDEs - uncomment on your machine if needed
#set(CMAKE_VERBOSE_MAKEFILE ON)
find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia OpenCV
find_package(OpenCV REQUIRED)
message("${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION} found in (${OpenCV_DIR})")
message("${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# Find Eigen3
find_package(Eigen3 REQUIRED)

# create the catkin package
catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_boss_library
    srrg2_property_library
    srrg2_config_library
    srrg2_point_cloud_library
    srrg2_matchable_library
    srrg2_image_library
    srrg2_messages_library
    srrg2_configurable_shell_library
    srrg2_data_structures_library
    srrg2_system_utils_library
    srrg2_viewer_core_library
    srrg2_viewer_library
  CATKIN_DEPENDS
    srrg_cmake_modules
  DEPENDS
    OpenCV
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
)

#ds help the catkin tool on 16.04
#ds (cmake seems unable to find single libraries,
#ds although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk everytime enable this
#ds so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})

#ds add project source code
add_subdirectory(src)

#ds apps
if (SRRG_COMPILE_APPS)
add_subdirectory(apps)
endif()

#ds test code
if (SRRG_COMPILE_TESTS)
add_subdirectory(tests)
endif()
