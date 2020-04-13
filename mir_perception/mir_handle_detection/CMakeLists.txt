cmake_minimum_required(VERSION 2.8.3)
project(mir_handle_detection)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
)

add_compile_options(-std=c++11
  -O3
  -march=native
)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drawer_handle_perceiver
  ros/src/drawer_handle_perceiver.cpp
)
target_link_libraries(drawer_handle_perceiver
  ${catkin_LIBRARIES}
)
