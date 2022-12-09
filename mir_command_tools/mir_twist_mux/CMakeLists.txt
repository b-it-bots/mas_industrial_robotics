cmake_minimum_required(VERSION 3.8)
project(mir_twist_mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(twist_mux REQUIRED)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/ros/)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/ros/)

ament_package()
