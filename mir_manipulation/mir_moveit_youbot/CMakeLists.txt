cmake_minimum_required(VERSION 2.8.3)
project(mir_moveit_youbot)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    mir_moveit_youbot_brsu_1
    mir_moveit_youbot_brsu_2
    mir_moveit_youbot_brsu_3
    mir_moveit_youbot_brsu_4
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(youbot-brsu-1)
  roslaunch_add_file_check(youbot-brsu-2)
  roslaunch_add_file_check(youbot-brsu-3)
  roslaunch_add_file_check(youbot-brsu-4)
endif()



### INSTALLS
install(DIRECTORY youbot-brsu-1
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/youbot-brsu-1
)
install(DIRECTORY youbot-brsu-2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/youbot-brsu-2
)
install(DIRECTORY youbot-brsu-3
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/youbot-brsu-3
)
install(DIRECTORY youbot-brsu-4
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/youbot-brsu-4
)
