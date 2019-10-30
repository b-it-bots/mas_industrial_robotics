cmake_minimum_required(VERSION 2.8.3)
project(mir_move_base)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    move_base_msgs
    actionlib
)

install(PROGRAMS
#  ros/scripts/simple_grasp_planner_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
#  add_rostest(ros/launch/simple_pregrasp_planner.test)
#  catkin_add_nosetests(ros/test/simple_pregrasp_planner_unittest.py)
endif()
