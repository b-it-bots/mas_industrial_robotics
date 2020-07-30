cmake_minimum_required(VERSION 2.8.3)
project(mir_workspace_alignment)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    tf
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)
