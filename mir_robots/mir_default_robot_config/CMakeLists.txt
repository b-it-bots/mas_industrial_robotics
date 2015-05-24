cmake_minimum_required(VERSION 2.8.3)
project(mir_default_robot_config)
find_package(catkin REQUIRED)

catkin_package()

set(robot_list
  youbot-brsu-1 
  youbot-brsu-2 
  youbot-brsu-3 
  youbot-brsu-4
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      roslaunch
  )

  roslaunch_add_file_check(upload_param.launch robot:=youbot-brsu-1)
endif()


### INSTALLS 
install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach(robot ${robot_list}) 
  install(DIRECTORY ${robot}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${robot}
  )
endforeach() 
