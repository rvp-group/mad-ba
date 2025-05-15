cmake_minimum_required(VERSION 2.8.3)
project(srrg_cmake_modules)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    cmake_modules
)