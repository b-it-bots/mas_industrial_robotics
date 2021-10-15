cmake_minimum_required(VERSION 2.8.3)
project(mir_grasp_verification)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    cv_bridge
)

find_package(OpenCV 3.2 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
