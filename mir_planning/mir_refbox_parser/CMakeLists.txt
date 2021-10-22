cmake_minimum_required(VERSION 2.8.3)
project(mir_refbox_parser)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    mir_planning_msgs
    tf
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/refbox_parser.test)
endif()


install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
install(PROGRAMS
  ros/scripts/refbox_parser_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
