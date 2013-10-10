cmake_minimum_required(VERSION 2.8.3)
project(mir_bringup)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cob_command_gui
    hokuyo_node
    mcr_default_env_config
    mir_arm_cartesian_control
    mir_default_robot_config
    mir_gripper_services
    mir_hardware_config
    mir_teleop
    openni_launch
    robot_state_publisher
    tf
    youbot_dashboard
    youbot_description 
    youbot_diagnostic_aggregator
    youbot_joint_trajectory_action
    youbot_oodl
)

set(
  INSTALL_DIRS 
    components
    robots
    tools
) 
foreach(dir ${INSTALL_DIRS}) 
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)

install(FILES robot.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  ) 
