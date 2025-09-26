cmake_minimum_required(VERSION 3.0.2)
project(mir_empty_space_prediction)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/emptyspace_prediction_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
