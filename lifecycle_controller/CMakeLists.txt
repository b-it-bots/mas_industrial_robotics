cmake_minimum_required(VERSION 3.5)

project(lifecycle_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(
  ros/include
  )

add_executable(lifecycle_controller
ros/src/lifecycle_controller.cpp)
ament_target_dependencies(lifecycle_controller
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
)

install(TARGETS
  lifecycle_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()