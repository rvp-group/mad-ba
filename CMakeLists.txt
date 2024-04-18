cmake_minimum_required(VERSION 3.0.2)
project(structure_refinement LANGUAGES CXX CUDA)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  srrg_cmake_modules
  srrg_hbst
  srrg2_core
  srrg2_core_ros
  srrg2_solver
  rviz_visual_tools
  rviz
)

set(SRRG_ENABLE_CUDA ON)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
message("CMAKE FLAGS SET TO ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CUDA_ARCH "${ARCH_FLAGS}" CACHE STRING "Value of the NVCC -arch option." FORCE)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH} -lineinfo")
message("CUDA ARCH FLAGS: ${ARCH_FLAGS}")
message("CUDA FLAGS: ${CUDA_NVCC_FLAGS}")
message(STATUS "CUDA_ARCH: \"${CUDA_ARCH}\"")



# find_package(QGLViewer REQUIRED)
# find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}: OpenCV version ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")

find_package(PCL REQUIRED)
# if(PCL_FOUND)
# message("PCL FOUND: building apps to dump point clouds in pcd format")
# endif()

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    structure_refinement
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg_hbst
    srrg2_core
    srrg2_core_ros
    srrg2_solver
    rviz_visual_tools
  DEPENDS
    # SRRG_QT
    # OpenCV
    # QGLViewer
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${QGLVIEWER_INCLUDE_DIR}
  # ${SRRG_QT_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${CUDA_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)
add_subdirectory(apps)