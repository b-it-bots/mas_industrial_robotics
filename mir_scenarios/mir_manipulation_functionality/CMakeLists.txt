cmake_minimum_required(VERSION 2.8.3)
project(mir_manipulation_functionality)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mir_bringup
    mir_bringup_sim
    mir_moveit_scene
    mir_states
    rospy
    smach
    smach_ros
    mcr_object_recognition_mean_circle
    mcr_common_converters
    mir_arm_motions
    mcr_trajectory_generation
    rostopic
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/manipulation_functionality.test)
endif()

### INSTALLS
install(PROGRAMS
  ros/scripts/manipulation_functionality
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
