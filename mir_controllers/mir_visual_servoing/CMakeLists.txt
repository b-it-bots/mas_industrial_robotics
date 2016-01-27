cmake_minimum_required(VERSION 2.8.3)
project(mir_visual_servoing)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mcr_image_filter
    mcr_blob_detection
    mcr_blob_tracking
    mcr_image_cartesian_mapper
    rostopic
    mcr_manipulation_measurers
    mcr_twist_limiter
    mcr_geometric_relation_monitors
    mcr_twist_demultiplexer
    mcr_twist_controller
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
