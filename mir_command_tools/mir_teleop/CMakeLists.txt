cmake_minimum_required(VERSION 3.5)
project(mir_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(brics_actuator REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(
  ros/include
)

add_executable(teleop_joypad_node
  ros/src/teleop_joypad.cpp
)
ament_target_dependencies(teleop_joypad_node
    rclcpp
    brics_actuator
    geometry_msgs
    joy
    sensor_msgs
    std_srvs
    moveit_msgs
)


### INSTALLS
install(TARGETS teleop_joypad_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY ros/launch/
  DESTINATION share/${PROJECT_NAME}/ros/launch
)
install(DIRECTORY ros/config/
  DESTINATION share/${PROJECT_NAME}/ros/config

)

ament_export_dependencies(
    rclcpp
    brics_actuator
    geometry_msgs
    joy
    sensor_msgs
    std_srvs
)
ament_package()
