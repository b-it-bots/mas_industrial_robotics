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
)

