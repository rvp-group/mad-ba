cmake_minimum_required(VERSION 2.8.3)
project(surfel_cloud_rviz_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  sensor_msgs
  roslib
)

find_package(PCL REQUIRED)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
add_definitions( ${QT_DEFINITIONS} )
include_directories( ${CMAKE_BINARY_DIR} )
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surfel_cloud_rviz_plugin
#  CATKIN_DEPENDS roscpp rviz sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Qt5Widgets_LIBRARIES}
)

add_library(surfel_cloud_rviz_plugin
  src/surfel_cloud_display.cpp
  src/point_cloud_common.cpp
  src/point_cloud.cpp
)

add_dependencies(surfel_cloud_rviz_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(surfel_cloud_rviz_plugin
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

