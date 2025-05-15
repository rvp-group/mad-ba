cmake_minimum_required(VERSION 2.8.3)
project(srrg2_qgl_viewport)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")
message("WARNING CXX FLAGS SET TO ${CMAKE_CXX_FLAGS}")

# ia we need Eigen, srrg2_core, QGL stuff
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_qgl_viewport_library
  CATKIN_DEPENDS
    srrg_cmake_modules
    srrg2_core
  DEPENDS
    QGLViewer
    GLUT
    OpenCV
)

include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${QGLViewer_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  )

link_directories(${catkin_LIBRARY_DIRS})

# set sources
add_subdirectory(src)
