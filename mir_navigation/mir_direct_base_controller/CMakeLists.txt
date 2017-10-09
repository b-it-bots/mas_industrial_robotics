cmake_minimum_required(VERSION 2.8.3)
project(mir_direct_base_controller)

find_package(catkin REQUIRED
)


catkin_package()

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()
