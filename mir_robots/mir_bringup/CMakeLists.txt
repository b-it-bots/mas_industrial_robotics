cmake_minimum_required(VERSION 3.8)
project(mir_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urg_node REQUIRED)
find_package(mir_hardware_config REQUIRED)
find_package(mir_teleop REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2 REQUIRED)
find_package(youbot_description REQUIRED)
find_package(youbot_driver_ros_interface REQUIRED)

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

### INSTALLS
set(
  INSTALL_DIRS
    components
    robots
    launch
)

foreach(robot ${robot_list})
  install(DIRECTORY ${robot}
    DESTINATION share/${PROJECT_NAME})
endforeach()

ament_package()
