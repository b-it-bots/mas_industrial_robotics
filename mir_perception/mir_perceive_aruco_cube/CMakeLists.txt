cmake_minimum_required(VERSION 2.8.3)
project(mir_perceive_aruco_cube)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    pcl_ros
    geometry_msgs
    tf
)

find_package(OpenCV 3.2 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    cv_bridge
    image_transport
    roscpp
  DEPENDS
    OpenCV
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_cube_perceiver
  ros/src/aruco_cube_perceiver.cpp
)
target_link_libraries(aruco_cube_perceiver
  ${catkin_LIBRARIES}
)
