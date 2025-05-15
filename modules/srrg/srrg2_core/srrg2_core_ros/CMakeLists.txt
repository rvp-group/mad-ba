cmake_minimum_required(VERSION 2.8.3)
project(srrg2_core_ros)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  tf
  image_transport
  std_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  rosbag
  rosbag_storage
  roscpp
  message_generation ##ia todo
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION} found in (${OpenCV_DIR})")
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_message_files(
  DIRECTORY msg
  FILES
  ViewerBufferMessage.msg
  PlannerStatusMessage.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_converters_library
    srrg2_messages_ros_library
    srrg2_viewer_core_ros_library ##ia todo
    srrg2_viewer_ros_library
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
    tf
    image_transport
    std_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    rosbag
    rosbag_storage
    roscpp
    message_runtime ##ia todo
  DEPENDS
    OpenCV
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

#ds help the catkin tool on 16.04
#ds (cmake seems unable to find single libraries,
#ds although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk everytime enable this
#ds so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
