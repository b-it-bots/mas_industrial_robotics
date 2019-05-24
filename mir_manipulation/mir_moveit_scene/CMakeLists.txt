cmake_minimum_required(VERSION 2.8.3)
project(mir_moveit_scene)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mas_perception_msgs
    moveit_msgs
    rospy
    shape_msgs
    std_msgs
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/arm_workspace_restricter_node
  ros/scripts/grasped_object_attacher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

