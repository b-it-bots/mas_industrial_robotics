cmake_minimum_required(VERSION 2.8.3)
project(mir_gripper_controller)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    dynamixel_controllers
    dynamixel_msgs
    roscpp
    roslint
    sensor_msgs
    std_msgs
    mcr_manipulation_msgs
)

catkin_package(
  CATKIN_DEPENDS
    dynamixel_controllers
    mcr_manipulation_msgs
    rospy
    sensor_msgs
    brics_actuator
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES ###
add_executable(dynamixel_gripper_node
  ros/src/dynamixel_gripper_node.cpp
)
add_dependencies(dynamixel_gripper_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_gripper_node
  ${catkin_LIBRARIES}
)


### TESTS
roslint_cpp()
roslint_python(ros/scripts/gripper_command_controller)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALL ###
install(PROGRAMS
  ros/scripts/gripper_command_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(
  TARGETS
    dynamixel_gripper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
