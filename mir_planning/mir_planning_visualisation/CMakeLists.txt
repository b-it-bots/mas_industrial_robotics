cmake_minimum_required(VERSION 2.8.3)
project(mir_planning_visualisation)

find_package(catkin REQUIRED
  COMPONENTS
  mir_planning_msgs
  mcr_gazebo_objects
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  mir_planning_msgs
)
