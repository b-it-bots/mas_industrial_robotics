cmake_minimum_required(VERSION 2.8.3)
project(mir_bringup_sim)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    gazebo_ros
    mcr_default_env_config
    mcr_gazebo_worlds   
    mir_arm_cartesian_control
    mir_arm_navigation  
    mir_default_robot_config
    mir_hardware_config
    mir_teleop
    pr2_controller_manager  
    youbot_gazebo_robots
    youbot_description
)

install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
)
install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

