cmake_minimum_required(VERSION 2.8.3)
project(mir_arm_kinematics_youbot)

find_package(catkin REQUIRED 
  COMPONENTS
    pluginlib
    roscpp
    tf_conversions
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    pluginlib
    moveit_core
)

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
