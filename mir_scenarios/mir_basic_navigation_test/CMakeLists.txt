cmake_minimum_required(VERSION 2.8.3)
project(mir_basic_navigation_test)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    cob_script_server
    mcr_default_env_config
    mir_bringup
    mir_bringup_sim
    mir_2dnav
    mir_common_states
    smach
    smach_ros
)


install(PROGRAMS
  ros/scripts/basic_navigation_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
