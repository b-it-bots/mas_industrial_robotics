cmake_minimum_required(VERSION 2.8.3)

project(mir_pddl_problem_generator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    rosplan_planning_system
    std_msgs
)

add_definitions(-std=c++11)
catkin_package(
    CATKIN_DEPENDS std_msgs
)

include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

# LIBRARY
add_library(pddl_problem_generator ros/src/pddl_problem_generator.cpp)

# EXECUTABLES
add_executable(pddl_problem_generator_node ros/src/pddl_problem_generator_node.cpp)
target_link_libraries(pddl_problem_generator_node ${catkin_LIBRARIES} pddl_problem_generator)

# TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

roslint_cpp()

# INSTALLS
install(DIRECTORY common/include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS pddl_problem_generator
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pddl_problem_generator_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
