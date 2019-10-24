cmake_minimum_required(VERSION 2.8.3)
project(mcr_knowledge)

find_package(catkin REQUIRED COMPONENTS
    mercury_planner
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS mercury_planner
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(PROGRAMS
  ros/scripts/upload_knowledge
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
