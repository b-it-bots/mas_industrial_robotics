cmake_minimum_required(VERSION 2.8.3)
project(mir_perceive_aruco_cube)

find_package(catkin REQUIRED
  COMPONENTS
    sensor_msgs
    cv_bridge
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
  DEPENDS
    OpenCV
)
