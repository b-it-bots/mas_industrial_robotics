cmake_minimum_required(VERSION 2.8.3)
project(mir_pregrasp_planning)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/PregraspPlannerParams.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    dynamic_reconfigure
    mcr_pose_generation
    mcr_manipulation_pose_selector
    mcr_manipulation_monitors
    mcr_arm_motions
    mcr_topic_tools
    std_msgs
    geometry_msgs
    visualization_msgs
    mcr_manipulation_msgs
    tf
    mcr_manipulation_measurers
    mcr_common_converters
)

### TESTS
roslint_python(
  ros/scripts/pose_mock_up_gui_node
  ros/scripts/pregrasp_planner_pipeline_node
  ros/scripts/simple_pregrasp_planner_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/simple_pregrasp_planner.test)
  add_rostest(ros/launch/pregrasp_planner_pipeline.test)
  catkin_add_nosetests(ros/test/simple_pregrasp_planner_unittest.py)
endif()

install(PROGRAMS
  ros/scripts/pose_mock_up_gui_node
  ros/scripts/pregrasp_planner_pipeline_node
  ros/scripts/simple_pregrasp_planner_node
  ros/config/PregraspPlannerParams.cfg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
