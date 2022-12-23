cmake_minimum_required(VERSION 3.8)
project(mir_perception_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mir_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)

include_directories(
  ros/include
  common/include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  ${cv_bridge_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${mir_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  common/src/bounding_box.cpp
  common/src/pointcloud_utils.cpp
  ros/src/bounding_box_visualizer.cpp
  ros/src/clustered_point_cloud_visualizer.cpp
  ros/src/label_visualizer.cpp
  ros/src/planar_polygon_visualizer.cpp
  ros/src/object_utils_ros.cpp
  ros/src/pointcloud_utils_ros.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  tf2_ros
  pcl_ros
  cv_bridge
  visualization_msgs
  std_msgs 
  sensor_msgs 
  geometry_msgs
)

install(
  DIRECTORY ros/include/${PROJECT_NAME}/ common/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
