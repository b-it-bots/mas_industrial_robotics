cmake_minimum_required(VERSION 3.5)
project(mir_object_recognition)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mas_perception_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(mir_object_segmentation REQUIRED)
#find_package(mir_perception_utils REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  ros/include
  common/include
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${mas_perception_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${mir_object_segmentation_INCLUDE_DIRS}
  ${mir_perception_utils_INCLUDE_DIRS}
  )

#add_executable(lc_talker ros/src/sample_lifecycle_pubsub.cpp)
#ament_target_dependencies(lc_talker rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs mas_perception_msgs)

add_executable(lifecycle_controller ros/src/lifecycle_controller.cpp)
target_link_libraries(lifecycle_controller
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
ament_target_dependencies(lifecycle_controller rclcpp std_msgs)

add_executable(mmor ros/src/multimodal_object_recognition.cpp)
ament_target_dependencies(mmor rclcpp rclcpp_lifecycle message_filters std_msgs sensor_msgs lifecycle_msgs mas_perception_msgs tf2_ros pcl_ros geometry_msgs cv_bridge)

# add_executable(segmenter ros/src/scene_segmentation_node.cpp)
# ament_target_dependencies(segmenter rclcpp std_msgs sensor_msgs mas_perception_msgs tf2_ros pcl_ros cv_bridge visualization_msgs)
# target_link_libraries(segmenter
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
# add_executable(transformer ros/src/sample_tf2_transformer.cpp)
# ament_target_dependencies(transformer rclcpp rclcpp_lifecycle message_filters std_msgs sensor_msgs lifecycle_msgs mas_perception_msgs tf2_ros pcl_ros geometry_msgs cv_bridge)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  lifecycle_controller
  #lc_talker
  mmor
  # transformer
  # segmenter
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  ros/launch/
  DESTINATION share/${PROJECT_NAME}/ros/launch
)

ament_package()