cmake_minimum_required(VERSION 2.8.3)
project(mir_conveyor_belt_test)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(PROGRAMS
  ros/scripts/conveyor_belt_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
