cmake_minimum_required(VERSION 2.8.3)
project(mir_visual_servoing)

find_package(catkin REQUIRED
  COMPONENTS
    brics_actuator
    cvblobs
    cv_bridge
    dynamic_reconfigure
    geometry_msgs  
    image_transport
    kdl_parser
    mir_controller_msgs
    moveit_msgs
    roscpp
    sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/VisualServoing.cfg
)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    dynamic_reconfigure
    geometry_msgs  
    mir_controller_msgs
    moveit_msgs
    sensor_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


### EXECUTABLES
add_executable(visual_servoing_node 
  ros/src/visual_servoing_node.cpp
  ros/src/visual_servoing_2d.cpp
)
add_dependencies(visual_servoing_node 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(visual_servoing_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS visual_servoing_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY common/data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/data
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
