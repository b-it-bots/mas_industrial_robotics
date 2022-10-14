cmake_minimum_required(VERSION 3.5)
project(mir_hardware_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(mcr_description REQUIRED)
find_package(mir_description REQUIRED)
find_package(youbot_description REQUIRED)

set(robot_list
  youbot-brsu-3
)

### TESTS
# if(CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)

#   roslaunch_add_file_check(upload_param.launch robot:=youbot-brsu-3)
# endif()


### INSTALLS
foreach(robot ${robot_list})
  install(DIRECTORY ${robot}
    DESTINATION share/${PROJECT_NAME}
  )
endforeach()

ament_package()
