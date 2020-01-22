cmake_minimum_required(VERSION 2.8.3)
project(mir_object_segmentation)

# Compile as C++11 for ROS Kinetic and newer
add_compile_options(-std=c++11
  -O3
  -march=native
)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    dynamic_reconfigure
    pcl_ros
    roscpp
    roslint
    tf
    mas_perception_msgs
    visualization_msgs
    mir_perception_utils
)

find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/SceneSegmentation.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    mas_perception_msgs
    visualization_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

add_definitions(-fpermissive)

### LIBRARIES ####################################################
add_library(${PROJECT_NAME}
  common/src/cloud_accumulation.cpp
  common/src/scene_segmentation.cpp
  ros/src/laserscan_segmentation.cpp
  ros/src/scene_segmentation_ros.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

### EXECUTABLES ###############################################
add_executable(scene_segmentation_node
  ros/src/scene_segmentation_node.cpp
)
add_dependencies(scene_segmentation_node
  ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
)
target_link_libraries(scene_segmentation_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

roslint_cpp()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS scene_segmentation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
