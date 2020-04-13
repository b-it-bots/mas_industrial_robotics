cmake_minimum_required(VERSION 2.8.3)
project(mir_moveit_client)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    mcr_moveit_client
)

### TESTS
roslint_add_test()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()
