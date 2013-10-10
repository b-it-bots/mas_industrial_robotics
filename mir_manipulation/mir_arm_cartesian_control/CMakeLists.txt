cmake_minimum_required(VERSION 2.8.3)
project(mir_arm_cartesian_control)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mcr_arm_cartesian_control
)

install(
  DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
