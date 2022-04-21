cmake_minimum_required(VERSION 3.5)
project(mir_object_recognition)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${mas_perception_msgs_INCLUDE_DIRS})

add_executable(lc_talker ros/src/sample_lifecycle_pubsub.cpp)
ament_target_dependencies(lc_talker rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs mas_perception_msgs)

add_executable(lc_client ros/src/sample_lifecycle_service_client.cpp)
target_link_libraries(lc_client
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

ament_target_dependencies(lc_client rclcpp std_msgs)


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
  lc_client
  lc_talker
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  ros/launch/
  DESTINATION share/${PROJECT_NAME}/ros/launch
)

ament_package()
