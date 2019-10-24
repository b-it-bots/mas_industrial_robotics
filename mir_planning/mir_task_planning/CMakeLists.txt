cmake_minimum_required(VERSION 2.8.3)
project(mir_task_planning)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib_msgs
  actionlib
  mir_planning_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  mir_planning_msgs
)
