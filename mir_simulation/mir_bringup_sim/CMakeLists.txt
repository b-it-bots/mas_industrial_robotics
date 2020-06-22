cmake_minimum_required(VERSION 2.8.3)
project(mir_bringup_sim)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    mcr_gazebo_worlds
    mir_arm_cartesian_control
    mir_default_robot_config
    mir_hardware_config
    robot_state_publisher
    youbot_gazebo_control
)

### TEST
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  #roslaunch_add_file_check(. robot:=youbot-brsu-1 robot_env:=brsu-c025)
  roslaunch_add_file_check(robots)
endif()


### INSTALLS
install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
)
install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
