cmake_minimum_required(VERSION 2.8.3)
project(mcr_cavity_detector)
set (CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    dynamic_reconfigure
    cv_bridge
    image_transport
    mas_perception_msgs
    tf
)
catkin_python_setup()
generate_dynamic_reconfigure_options(
  ros/config/CavityFinder.cfg
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package()


include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



### EXECUTABLES ###############################################
add_executable(cavity_finder
  ros/src/cavity_finder_node.cpp
  ros/src/cavity_finder_ros.cpp
  common/src/cavity_finder.cpp
)

add_dependencies(cavity_finder
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(cavity_finder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(TARGETS cavity_finder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY common/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

