cmake_minimum_required(VERSION 2.8.3)
project(mir_gripper_controller)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mir_manipulation_msgs
    rospy
    sensor_msgs
    brics_actuator
)
install(PROGRAMS
  ros/scripts/gripper_command_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
