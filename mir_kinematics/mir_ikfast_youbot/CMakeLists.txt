cmake_minimum_required(VERSION 2.8.3)
project(mir_ikfast_youbot)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

find_package(Eigen REQUIRED)
find_package(LAPACK REQUIRED)


include_directories(
  SYSTEM
  common/include/${PROJECT_NAME}
  common/src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  LIBRARIES
  DEPENDS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)


set(IKFAST_LIBRARY_NAME youbot_arm_1_moveit_ikfast_plugin)


add_library(${IKFAST_LIBRARY_NAME} 
  ros/src/youbot_arm_1_ikfast_moveit_plugin.cpp
)
target_link_libraries(${IKFAST_LIBRARY_NAME} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${LAPACK_LIBRARIES}
)


install(TARGETS ${IKFAST_LIBRARY_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/ 
  DESTINATION include
)
install(FILES youbot_moveit_ikfast_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
