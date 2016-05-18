cmake_minimum_required(VERSION 2.8.3)
project(mir_basic_transportation_test)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mcr_gazebo_objects
    mcr_linear_regression
    mcr_object_recognition_bounding_box
    mcr_twist_synchronizer
    mir_2dnav
    mir_base_placement
    mir_bringup
    mir_bringup_sim
    mir_moveit_scene
    mir_states
    mir_visual_servoing
    rospy
    smach
    smach_ros
    mir_arm_motions
    mcr_object_recognition_mean_circle
    mcr_manipulation_measurers
    mcr_common_converters
    mir_grasp_monitors
    rostopic
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/basic_transportation_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
