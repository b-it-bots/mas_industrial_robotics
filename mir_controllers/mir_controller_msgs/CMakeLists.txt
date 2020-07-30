cmake_minimum_required(VERSION 2.8.3)
project(mir_controller_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    roscpp
    std_msgs
)

add_message_files(
  FILES
    VisualServoingState.msg
)

add_service_files(
  FILES
    StartVisualServoing.srv
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
