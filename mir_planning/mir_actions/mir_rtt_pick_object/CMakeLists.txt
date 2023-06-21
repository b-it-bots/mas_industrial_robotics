cmake_minimum_required(VERSION 2.8.3)
project(mir_rtt_pick_object)

find_package(catkin REQUIRED
  COMPONENTS
  mas_perception_msgs
)

catkin_package(
  CATKIN_DEPENDS
  mas_perception_msgs
)
