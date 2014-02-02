cmake_minimum_required(VERSION 2.8.3)
project(mir_generic_states)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ros/include
  CATKIN_DEPENDS
    actionlib
    cob_script_server
    geometry_msgs
    mir_base_placement
    move_base_msgs
    rospy
    smach
    smach_ros
    std_srvs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)