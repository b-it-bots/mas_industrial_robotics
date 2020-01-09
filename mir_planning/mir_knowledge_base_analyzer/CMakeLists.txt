cmake_minimum_required(VERSION 2.8.3)
project(mir_knowledge_base_analyzer)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    roscpp
    rosplan_knowledge_msgs
    mir_planning_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ros/include

)

### EXECUTABLES ###
add_executable(knowledge_base_queries
  ros/src/knowledge_base_queries.cpp
)
add_dependencies(knowledge_base_queries
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(knowledge_base_queries
  ${catkin_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(
  TARGETS
    knowledge_base_queries
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
