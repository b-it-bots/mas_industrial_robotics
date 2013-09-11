cmake_minimum_required(VERSION 2.8.3)
project(mir_description)
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/
)