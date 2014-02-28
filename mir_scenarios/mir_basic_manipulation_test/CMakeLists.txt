cmake_minimum_required(VERSION 2.8.3)
project(mir_basic_manipulation_test)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    cob_script_server
    mcr_default_env_config
    mcr_gazebo_objects
    mcr_relative_movements
    mir_2dnav
    mir_base_placement
    mir_bringup
    mir_bringup_sim
    mir_states
    mir_object_detection
    rospy
    smach
    smach_ros
    mir_visual_servoing
)


install(PROGRAMS
  ros/scripts/basic_manipulation_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
