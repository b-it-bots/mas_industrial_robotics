cmake_minimum_required(VERSION 3.8)
project(mir_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes/)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf/)

ament_package()
