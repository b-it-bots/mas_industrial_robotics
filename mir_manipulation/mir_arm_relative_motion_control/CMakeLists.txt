cmake_minimum_required(VERSION 2.8.3)
project(mir_arm_relative_motion_control)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rospy
    moveit_commander
    moveit_msgs
    geometry_msgs
    std_msgs
    tf
    actionlib
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/arm_relative_motion_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
