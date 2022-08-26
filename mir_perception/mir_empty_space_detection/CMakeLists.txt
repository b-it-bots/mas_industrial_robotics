cmake_minimum_required(VERSION 2.8.3)
project(mir_empty_space_detection)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  mir_object_segmentation
  mir_perception_utils
)

generate_dynamic_reconfigure_options(
  ros/config/EmptySpaceDetection.cfg
)

add_compile_options(
  -O3
)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(empty_space_detector
  ros/src/empty_space_detector.cpp
)

add_dependencies(empty_space_detector
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(empty_space_detector
  ${catkin_LIBRARIES}
)
