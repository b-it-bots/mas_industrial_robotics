cmake_minimum_required(VERSION 2.8.3)
project(mir_grasp_monitors)

find_package(catkin REQUIRED
  COMPONENTS
    dynamixel_msgs
    roscpp
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    dynamixel_msgs
    mir_manipulation_msgs
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES ###
add_executable(dynamixel_gripper_grasp_monitor_node
  ros/src/dynamixel_gripper_grasp_monitor_node.cpp
)
add_dependencies(dynamixel_gripper_grasp_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_gripper_grasp_monitor_node
  ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALL ###
install(
  TARGETS
    dynamixel_gripper_grasp_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    ros/scripts/youbot_gripper_grasp_monitor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
