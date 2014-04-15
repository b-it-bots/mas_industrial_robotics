cmake_minimum_required(VERSION 2.8.3)
project(mir_teleop)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    brics_actuator
    geometry_msgs
    joy
    moveit_msgs
    roscpp
    sensor_msgs
    std_srvs
)

find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/TeleopJoypad.cfg
)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    geometry_msgs
    mcr_teleop
    moveit_msgs
    pr2_teleop
    sensor_msgs
    std_srvs
)

include_directories(
  ros/include
)

add_executable(teleop_joypad_node 
  ros/src/teleop_joypad_node.cpp
  ros/src/teleop_joypad.cpp
)
add_dependencies(teleop_joypad_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(teleop_joypad_node
  ${catkin_LIBRARIES}
)

install(TARGETS teleop_joypad_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
install(DIRECTORY ros/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
