cmake_minimum_required(VERSION 2.8.3)
project(mir_planning_core)

find_package(catkin REQUIRED
  COMPONENTS
  mir_planning_msgs
)

catkin_package(
  CATKIN_DEPENDS
  mir_planning_msgs
)
