cmake_minimum_required(VERSION 2.8.3)
project(mir_basic_manipulation_test)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mir_basic_transportation_test
    mir_states
)


install(PROGRAMS
  ros/scripts/basic_manipulation_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
