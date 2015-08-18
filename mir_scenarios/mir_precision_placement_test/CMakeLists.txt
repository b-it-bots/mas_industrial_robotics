cmake_minimum_required(VERSION 2.8.3)
project(mir_precision_placement_test)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mir_basic_transportation_test
)


### INSTALLS
install(PROGRAMS
  ros/scripts/precision_placement_test
  ros/scripts/combined_btt_ppt_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
