cmake_minimum_required(VERSION 2.8.3)
project(mir_actions)

find_package(catkin REQUIRED
    COMPONENTS
    mir_move_base_safe
    mir_perceive_location
    mir_perceive_cavity
    mir_pick_object
    mir_place_object
    mir_stage_object
    mir_unstage_object
    mir_insert_object
    mir_insert_cavity
    mir_planning_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    mir_planning_msgs
)
