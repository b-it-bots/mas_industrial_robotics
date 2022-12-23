cmake_minimum_required(VERSION 3.8)
project(mir_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(manipulation_msg_files
  "mir_manipulation/msg/GripperCommand.msg"
)

set(perception_msg_files
  "mir_perception/msg/BoundingBox.msg"
  "mir_perception/msg/BoundingBoxList.msg"
  "mir_perception/msg/ImageList.msg"
  "mir_perception/msg/LaserScanSegment.msg"
  "mir_perception/msg/LaserScanSegmentList.msg"
  "mir_perception/msg/Object.msg"
  "mir_perception/msg/ObjectEmbedding.msg"
  "mir_perception/msg/ObjectList.msg"
  "mir_perception/msg/ObjectView.msg"
  "mir_perception/msg/PlanarPolygon.msg"
  "mir_perception/msg/NamedShape.msg"
  "mir_perception/msg/NamedColor.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${manipulation_msg_files}
  ${perception_msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
