cmake_minimum_required(VERSION 2.8.3)
project(mir_2dexplore)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gmapping
    cob_scan_unifier
    move_base
    explore_lite
    frontier_exploration
)
