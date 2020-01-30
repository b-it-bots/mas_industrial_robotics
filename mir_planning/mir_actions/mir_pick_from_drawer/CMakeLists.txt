cmake_minimum_required(VERSION 2.8.3)
project(mir_pick_from_drawer)

find_package(catkin REQUIRED
  COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

add_compile_options(-std=c++11
  -O3
  -march=native
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drawer_handle_perceiver ros/src/drawer_handle_perceiver.cpp)
target_link_libraries(drawer_handle_perceiver ${catkin_LIBRARIES})
