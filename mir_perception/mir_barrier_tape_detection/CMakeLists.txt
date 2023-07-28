cmake_minimum_required(VERSION 2.8.3)
project(mir_barrier_tape_detection)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    image_transport
    roscpp
    sensor_msgs
    cv_bridge
    pcl_ros
    message_filters
    geometry_msgs
    tf
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_BUILD_TYPE Release)

generate_dynamic_reconfigure_options(
  ros/config/BarrierTapeDetection.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    barrier_tape_detection
  CATKIN_DEPENDS
    dynamic_reconfigure
    image_transport
    roscpp
    sensor_msgs
    cv_bridge
  DEPENDS
    OpenCV
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(barrier_tape_detection
  common/src/barrier_tape_detection.cpp
)

add_dependencies(barrier_tape_detection
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(barrier_tape_detection
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(barrier_tape_detection_node
  ros/src/barrier_tape_detection_ros.cpp
)

add_dependencies(barrier_tape_detection_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(barrier_tape_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  barrier_tape_detection
)

install(
  TARGETS
    barrier_tape_detection
    barrier_tape_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
