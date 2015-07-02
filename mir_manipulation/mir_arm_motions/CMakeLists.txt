cmake_minimum_required(VERSION 2.8.3)
project(mir_arm_motions)

find_package(catkin REQUIRED)

catkin_package()

###TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
