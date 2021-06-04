cmake_minimum_required(VERSION 2.8.3)
project(mir_atwork_commander_client)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  atwork_commander_msgs
)
