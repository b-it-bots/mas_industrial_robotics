cmake_minimum_required(VERSION 2.8.3)
project(mir_closed_loop_pick_base_controller)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)
