cmake_minimum_required(VERSION 3.0.2)
project(structure_refinement LANGUAGES CXX)

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

set(SRRG_ENABLE_CUDA OFF)
include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
message("CMAKE FLAGS SET TO ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(QGLViewer REQUIRED)
# find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}: OpenCV version ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")

find_package(PCL REQUIRED COMPONENTS)
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
    # OpenMP
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
  ${OpenMP_LIBS}
)


link_directories(${catkin_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_subdirectory(src)
add_subdirectory(apps)