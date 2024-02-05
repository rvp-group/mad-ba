cmake_minimum_required(VERSION 3.0.2)
project(structure_refinement)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg_hbst
  srrg2_core
  srrg2_core_ros
  srrg2_solver
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic -fopenmp")
message("CMAKE FLAGS SET TO ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(QGLViewer REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}: OpenCV version ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# hbst stuff
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")

# optional
# PCL only required read or dump data in universal way
find_package(PCL REQUIRED)
if(PCL_FOUND)
message("PCL FOUND: building apps to dump point clouds in pcd format")
endif()

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    md_slam
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg_hbst
    srrg2_core
    srrg2_core_ros
    srrg2_solver
  DEPENDS
    SRRG_QT
    OpenCV
    QGLViewer
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${QGLVIEWER_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)
add_subdirectory(apps)