cmake_minimum_required(VERSION 2.8.3)
project(mir_gripper_controller)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    mcr_manipulation_msgs
    roscpp
    roslint
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    mcr_manipulation_msgs
    rospy
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALL ###
#install(PROGRAMS
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
