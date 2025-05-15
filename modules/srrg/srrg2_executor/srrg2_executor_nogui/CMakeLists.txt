cmake_minimum_required(VERSION 2.8.3)
project(srrg2_executor_nogui)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_core_ros
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
# ROS inclusion prevents pedanticness
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic")

find_package(Eigen3 REQUIRED)
#ia find all packages
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION}")
message(STATUS "${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
#  INCLUDE_DIRS no_include_dirs
#  LIBRARIES no_srrg2_executor_library

 CATKIN_DEPENDS
    srrg2_core
    srrg2_core_ros
    
DEPENDS
    OpenCV
    #QGLViewer
)


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

link_directories(${catkin_LIBRARY_DIRS})

add_executable(srrg2_shell src/apps/srrg2_shell.cpp)
target_link_libraries(srrg2_shell
  ${catkin_LIBRARIES})

