cmake_minimum_required(VERSION 3.16)
project(mir_object_recognition)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# temporary linker flags for build, remove or comment accordingly 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")


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
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# message(${CMAKE_INSTALL_PREFIX})
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
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${mir_object_segmentation_INCLUDE_DIRS}
  ${mir_perception_utils_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/../cv_bridge/include
  )
include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# message(${cv_bridge_INCLUDE_DIRS})
# message(${mas_perception_msgs_INCLUDE_DIRS})
# set(dependencies OpenCV cv_bridge)

#add_executable(lc_talker ros/src/sample_lifecycle_pubsub.cpp)
#ament_target_dependencies(lc_talker rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs mas_perception_msgs)

add_executable(lifecycle_controller ros/src/lifecycle_controller.cpp)
target_link_libraries(lifecycle_controller
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
ament_target_dependencies(lifecycle_controller rclcpp std_msgs)

add_library( percption_visuliazation
  common/src/bounding_box.cpp
  common/src/pointcloud_utils.cpp
  ros/src/mir_perception_utils/bounding_box_visualizer.cpp
  ros/src/mir_perception_utils/clustered_point_cloud_visualizer.cpp
  ros/src/mir_perception_utils/label_visualizer.cpp
  ros/src/mir_perception_utils/planar_polygon_visualizer.cpp
  ros/src/mir_perception_utils/object_utils_ros.cpp
  ros/src/mir_perception_utils/pointcloud_utils_ros.cpp
)

ament_target_dependencies( percption_visuliazation
 tf2_ros
 pcl_ros
)

add_library( scene_segmentation
  common/src/cloud_accumulation.cpp
  common/src/scene_segmentation.cpp
  ros/src/mir_object_segmentation/laserscan_segmentation.cpp
  ros/src/mir_object_segmentation/scene_segmentation_ros.cpp
  #ros/src/mir_object_segmentation/scene_segmentation_node.cpp
)

target_link_libraries(scene_segmentation
  percption_visuliazation
)

ament_target_dependencies(scene_segmentation 
  tf2_ros 
  pcl_ros
)

add_executable(mmor ros/src/multimodal_object_recognition.cpp)
ament_target_dependencies(mmor 
  rclcpp
  rclcpp_lifecycle 
  message_filters 
  std_msgs 
  sensor_msgs 
  lifecycle_msgs 
  mas_perception_msgs 
  tf2_ros 
  pcl_ros 
  geometry_msgs 
  OpenCV
  cv_bridge
)
target_link_libraries(mmor
  percption_visuliazation
  scene_segmentation
)


# add_executable(transformer ros/src/sample_tf2_transformer.cpp)
# ament_target_dependencies(transformer rclcpp rclcpp_lifecycle message_filters std_msgs sensor_msgs lifecycle_msgs mas_perception_msgs tf2_ros pcl_ros geometry_msgs)

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
  # lc_talker
  mmor
  # transformer
  # segmenter
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  ros/launch/
  DESTINATION share/${PROJECT_NAME}/ros/launch
)

install(DIRECTORY
  ros/config/
  DESTINATION share/${PROJECT_NAME}/ros/config
)

ament_package()