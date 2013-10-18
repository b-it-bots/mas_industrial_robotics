cmake_minimum_required(VERSION 2.8.3)
project(mir_kinect_calibration)

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

#...: enable all but certain warnings
add_definitions(-Wall)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)
add_definitions(-fpermissive)


find_package(catkin REQUIRED
  COMPONENTS
    #arm_navigation_msgs   <!-- need to be changed to moveit, but this involves more changes -->
    actionlib
    geometry_msgs
    pcl_ros
    rospy
    tf
)

find_package(PCL 1.6 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    #arm_navigation_msgs   <!-- need to be changed to moveit, but this involves more changes -->
    actionlib
    geometry_msgs
    rospy
    tf
)

include_directories(
  common/include
  ros/include
  ${PCL_INCLUDE_DIRS}  
)

add_library(transform_estimation
  common/src/transformation_estimation.cpp
)
add_dependencies(transform_estimation
  ${catkin_EXPORTED_TARGETS}
)

install(
  PROGRAMS
    ros/scripts/calibrate_kinect
    ros/scripts/verify_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS transform_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

