cmake_minimum_required(VERSION 2.8.3)
project(mir_2dslam)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gmapping
    cob_scan_unifier
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
