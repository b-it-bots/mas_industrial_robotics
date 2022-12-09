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
find_package(mir_gripper_controller REQUIRED)
find_package(youbot_description REQUIRED)
find_package(youbot_driver_ros_interface REQUIRED)
find_package(mir_hardware_config REQUIRED)
find_package(mir_twist_mux REQUIRED)

### INSTALLS
set(
  INSTALL_DIRS
    components
    robots
    launch
)

foreach(dir ${INSTALL_DIRS})
  install(DIRECTORY ${dir}
    DESTINATION share/${PROJECT_NAME})
endforeach()

ament_package()
