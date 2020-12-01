cmake_minimum_required(VERSION 2.8.3)
project(mir_object_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11
  -O3
  -march=native
)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    dynamic_reconfigure
    mas_perception_msgs
    pcl_ros
    roscpp
    rospy
    roslint
    tf
    visualization_msgs
    geometry_msgs
    mir_object_segmentation
    mir_perception_utils
)
catkin_python_setup()
find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/SceneSegmentation.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  LIBRARIES
    mir_object_segmentation
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
  ros/src/multimodal_object_recognition_utils.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

### EXECUTABLES ###############################################
add_executable(multimodal_object_recognition
  ros/src/multimodal_object_recognition_node.cpp
)
add_dependencies(multimodal_object_recognition
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(multimodal_object_recognition
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

roslint_cpp()

### INSTALLS
install(PROGRAMS
  ros/scripts/pc_object_recognizer_node
  ros/scripts/rgb_object_recognizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/config
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
install(DIRECTORY ros/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/scripts
)
