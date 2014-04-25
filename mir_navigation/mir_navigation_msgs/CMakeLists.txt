cmake_minimum_required(VERSION 2.8.3)
project(mir_navigation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    message_generation
    std_msgs
)

add_action_files(
  DIRECTORY action
  FILES OrientToBase.action
)

generate_messages(
  DEPENDENCIES 
    actionlib_msgs 
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime    
    std_msgs
)
