cmake_minimum_required(VERSION 2.8.3)
project(mir_gripper_services)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    mir_manipulation_msgs
    rospy
    sensor_msgs
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      roslaunch
  )

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/gripper_service_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)    
