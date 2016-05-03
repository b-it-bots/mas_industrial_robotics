cmake_minimum_required(VERSION 2.8.3)
project(mir_states)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    mcr_states
    mir_base_placement
    move_base_msgs
    rospy
    smach
    smach_ros
    std_srvs
    tf
)
