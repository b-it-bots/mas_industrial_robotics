cmake_minimum_required(VERSION 2.8.3)
project(mir_move_base_safe)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
    nav_msgs
)
