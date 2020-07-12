cmake_minimum_required(VERSION 2.8.3)
project(mir_manipulation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    roscpp
    rospy
    std_msgs
)

add_service_files(
  FILES
    IsGripperClosed.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
