cmake_minimum_required(VERSION 2.8.3)
project(mir_perception_utils)

# Compile as C++11 for ROS Kinetic and newer
add_compile_options(-std=c++11
  -O3
  -march=native
)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    mas_perception_msgs
    pcl_ros
    roscpp
    roslint
    tf
    visualization_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

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
  common/src/bounding_box.cpp
  ros/src/object_utils_ros.cpp
  ros/src/pointcloud_utils_ros.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

roslint_cpp()

### TESTS
#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(ros/launch)
#endif()

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


install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
