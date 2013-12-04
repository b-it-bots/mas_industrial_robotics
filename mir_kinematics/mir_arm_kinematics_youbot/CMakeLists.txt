cmake_minimum_required(VERSION 2.8.3)
project(mir_arm_kinematics_youbot)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

find_package(orocos_kdl REQUIRED)

include_directories(
  ros/include
  common/include
  common/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
)

catkin_package(
  DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)


set(IKFAST_LIBRARY_NAME arm_analytical_inverse_kinematics_youbot_moveit_plugin)

add_library(${IKFAST_LIBRARY_NAME} 
  ros/src/arm_analytical_inverse_kinematics_youbot_moveit_plugin.cpp
  ros/src/arm_analytical_inverse_kinematics.cpp
)
target_link_libraries(${IKFAST_LIBRARY_NAME} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${LAPACK_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)


install(TARGETS ${IKFAST_LIBRARY_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
