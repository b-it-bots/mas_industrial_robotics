cmake_minimum_required(VERSION 2.8.3)
project(mir_moveit_to_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    mcr_manipulation_utils_ros
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

