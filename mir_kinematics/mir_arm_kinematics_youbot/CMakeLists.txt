cmake_minimum_required(VERSION 2.8.3)
project(mir_arm_kinematics_youbot)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(
  ros/include
  common/include
  common/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package()

add_library(arm_analytical_inverse_kinematics_youbot_moveit_plugin
  ros/src/arm_analytical_inverse_kinematics_youbot_moveit_plugin.cpp
  ros/src/arm_analytical_inverse_kinematics.cpp
)

target_link_libraries(arm_analytical_inverse_kinematics_youbot_moveit_plugin
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${LAPACK_LIBRARIES}
)


install(TARGETS arm_analytical_inverse_kinematics_youbot_moveit_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
