cmake_minimum_required(VERSION 3.0.2)
project(mad_ba LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  srrg_cmake_modules
  srrg2_core
  srrg2_core_ros
  srrg2_solver
)

set(SRRG_ENABLE_CUDA OFF)
include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
message("CMAKE FLAGS SET TO ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "${PROJECT_NAME}: OpenCV version ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS")

find_package(PCL REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    mad_ba
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
    srrg2_core_ros
    srrg2_solver
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${OpenMP_LIBS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_subdirectory(src)
add_subdirectory(apps)
