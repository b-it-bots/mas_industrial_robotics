cmake_minimum_required(VERSION 3.0.2)
project(mir_ppt_detection)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    pcl_conversions
    roscpp
    rospy
    std_msgs
    sensor_msgs
    mas_perception_msgs
)

add_message_files(
    FILES
        Cavity.msg
        Cavities.msg)

generate_messages(
    DEPENDENCIES
        sensor_msgs
        std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/include/eigen3) # FIXME
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported/)

add_executable(ppt_detector
    src/ppt_detector.cpp
    src/min_distance_to_hull_calculator.cpp
)
target_link_libraries(ppt_detector
    ${catkin_LIBRARIES}
    yaml-cpp
)
add_dependencies(ppt_detector mir_ppt_detection_generate_messages_cpp)
