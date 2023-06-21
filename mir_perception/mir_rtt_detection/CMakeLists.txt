cmake_minimum_required(VERSION 2.8.3)
project(mir_rtt_detection)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(
  -O3
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
find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/SceneSegmentation.cfg
)

catkin_package(

  LIBRARIES
    mir_object_segmentation
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    mas_perception_msgs
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

add_definitions(-fpermissive)

### LIBRARIES ####################################################

### EXECUTABLES ###############################################

### INSTALLS

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
install(DIRECTORY ros/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/scripts
)
