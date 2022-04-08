cmake_minimum_required(VERSION 2.8.3)
project(mir_waypoint_trajectory_generation)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    std_msgs
    geometry_msgs
    moveit_msgs
    brics_actuator
    moveit_commander
)

### TESTS
roslint_python(
  ros/scripts/waypoint_trajectory_generation_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
endif()

install(PROGRAMS
  ros/scripts/waypoint_trajectory_generation_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
