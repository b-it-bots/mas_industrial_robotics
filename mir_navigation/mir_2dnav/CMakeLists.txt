cmake_minimum_required(VERSION 2.8.3)
project(mir_2dnav)

find_package(catkin REQUIRED)

catkin_package()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch "robot:=youbot-brsu-1 robot_env:=brsu-c025")
endif()

### INSTALLS
install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

