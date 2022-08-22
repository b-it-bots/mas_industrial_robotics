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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold -fPIC")


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
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp)

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
  ${visualization_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${YAML_CPP_VENDOR_INCLUDE_DIRS}
  ${mir_object_segmentation_INCLUDE_DIRS}
  ${mir_perception_utils_INCLUDE_DIRS}
  $ENV{COLCON_PREFIX_PATH}/cv_bridge/include
  )
include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(perception_visualization
  common/src/bounding_box.cpp
  common/src/pointcloud_utils.cpp
  common/src/cloud_accumulation.cpp
  # common/src/scene_segmentation.cpp
  ros/src/mir_perception_utils/bounding_box_visualizer.cpp
  ros/src/mir_perception_utils/clustered_point_cloud_visualizer.cpp
  ros/src/mir_perception_utils/label_visualizer.cpp
  ros/src/mir_perception_utils/planar_polygon_visualizer.cpp
  ros/src/mir_perception_utils/object_utils_ros.cpp
  ros/src/mir_perception_utils/pointcloud_utils_ros.cpp
  ros/src/multimodal_object_recognition_utils.cpp
  # ros/src/mir_object_segmentation/laserscan_segmentation.cpp
  # ros/src/mir_object_segmentation/scene_segmentation_node.cpp
  # ros/src/mir_object_segmentation/scene_segmentation_ros.cpp
)

ament_target_dependencies(perception_visualization 
  tf2_ros
  pcl_ros
  cv_bridge
  visualization_msgs)

add_library( scene_segmentation
  common/src/cloud_accumulation.cpp
  common/src/scene_segmentation.cpp
  ros/src/mir_object_segmentation/laserscan_segmentation.cpp
  ros/src/mir_object_segmentation/scene_segmentation_ros.cpp
  #ros/src/mir_object_segmentation/scene_segmentation_node.cpp
)

target_link_libraries(scene_segmentation
  perception_visualization
)

ament_target_dependencies(scene_segmentation 
  tf2_ros 
  pcl_ros
  cv_bridge
  visualization_msgs
)

add_library(mmor SHARED
            ros/src/multimodal_object_recognition.cpp
            ros/src/multimodal_object_recognition_parameters.cpp
            )

target_compile_definitions(mmor
            PRIVATE "COMPOSITION_BUILDING_DLL")          
# add_executable(mmor ros/src/multimodal_object_recognition.cpp)
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
  visualization_msgs
  OpenCV
  cv_bridge
  rclcpp_components
  yaml_cpp_vendor
)
target_link_libraries(mmor
  perception_visualization
  scene_segmentation
)
rclcpp_components_register_nodes(mmor "perception_namespace::MultiModalObjectRecognitionROS")

add_library(data_collector SHARED
            ros/src/mmor_data_collector.cpp
            ros/src/multimodal_object_recognition.cpp
            ros/src/multimodal_object_recognition_parameters.cpp
            )

target_compile_definitions(data_collector
            PRIVATE "COMPOSITION_BUILDING_DLL")          
# add_executable(mmor ros/src/multimodal_object_recognition.cpp)
ament_target_dependencies(data_collector
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
  rclcpp_components
  yaml_cpp_vendor
)
target_link_libraries(data_collector
  perception_visualization
  scene_segmentation
)
rclcpp_components_register_nodes(data_collector "perception_namespace::DataCollector")

ament_export_include_directories(include)
# ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_node)

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
  mmor
  data_collector

  EXPORT export_mmor
  EXPORT export_data_collector

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  mmor
  data_collector

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