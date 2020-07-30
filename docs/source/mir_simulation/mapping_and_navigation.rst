.. _mapping_and_navigaiton:

Mapping and Navigation for Simulation
=====================================


Mapping
-------

Run simulation related nodes

* Run roscore

  .. code-block:: bash

    roscore

* Launch the robot (In another terminal)

  .. code-block:: bash

    roslaunch mir_bringup_sim robot.launch

* Run gazebo simulator (In another terminal)

  .. code-block:: bash

    rosrun gazebo_ros gzclient

* Run rviz (In another terminal)

  .. code-block:: bash

    rosrun rviz rviz

  .. note::

    To setup the RViz, please go to the bottom of this page.

Generate map
------------

* Run 2D SLAM (In another terminal)

  .. code-block:: bash

    roslaunch mir_2dslam 2dslam.launch


  Now there should be robot in an empty map in RViz.

* Move the robot around the map. (In another terminal)

  .. code-block:: bash

    roslaunch mir_teleop teleop_keyboard.launch

  WSAD keys set the robot in motion and "Space bar" stops that motion.

  As you move the robot around, you should be able to see walls appearing in the map
  in RViz and all the other area will be free. After you have mapped the whole environment,
  you can save the map in config map directory.

* Move to map config directory (In another terminal)

  .. code-block:: bash

    roscd mcr_default_env_config

* Then make a directory and move inside that newly created directory

  .. code-block:: bash

    mkdir test_map
    cd test_map

* Now you can save the map that you just created

  .. code-block:: bash

    rosrun map_server map_saver

  This will ideally create 2 files namely *map.pgm* and *map.yml*.
  Now you can exit out of *mir_2dslam* execution. You can also exit from *mir_teleop, gazebo, mir_bringup_sim*

Making the map usable
----------------------

In order to use this map in future to navigate, follow the following steps:

* Add files where the goals will be saved \(in the same directory where the map files have been saved\)

  .. code-block:: bash

    touch navigation_goals.yaml
    touch orientation_goals.yaml

* Make a copy of the existing launch file.

  .. code-block:: bash

    cd ~/catkin_ws/src/mas_common_robotics/mcr_environments/mcr_gazebo_worlds/ros/launch
    cp brsu-c025-sim.launch test_map.launch

  .. note::

    Inside *test_map.launch*, edit the argument of **robot_env** (line 10).
    Replace *brsu-c025-sim* with *test_map*. Save this file.

* Make a copy of xacro file.

  .. code-block:: bash

    cd ~/catkin_ws/src/mas_common_robotics/mcr_environments/mcr_gazebo_worlds/common/worlds/
    cp brsu-c025-sim.urdf.xacro test_map.urdf.xacro

.. note::

  Now your newly created map should be ready for use.

Navigation
-----------

* Run the commands from "Run simulation related nodes" as mentioned above to bring the robot up.

* Launch the navigation node

  .. code-block:: bash

    roslaunch mir_2dnav 2Dnav.launch

* Open RViz

  Add *PoseArray* in RViz and change its topic to */particlecloud*.
  Now you will be able to see red arrows around the robot. These arrow show the pose of the robot.

* You now need to localize the robot to get its correct pose. Move the robot around the map. (In another terminal)

  .. code-block:: bash

    roslaunch mir_teleop teleop_keyboard.launch

  Rotate the robot in its place using QE keys.
  You will notice the red arrows converging around the robot. Once the the robot is reasonably localised,
  you can navigate the robot around in 2 ways:

  1. GUI (RViz)

    Click on the `2D Nav Goal` and select a goal on the map.

  2. Terminal based (ROS Action Server Client)

    For Action server client, the robot first needs name of position for source and
    destination places (It cannot use `x, y, theta` values)

    To name the poses, you have to execute *save_base_map_poses_to_file*

    .. code-block:: bash

      roscd mcr_default_env_config
      cd test_map
      rosrun mcr_navigation_tools save_base_map_poses_to_file

    This program is terminal based interactive program. The program will ask you to name the position.
      1. You can now navigate the robot to your desired position (using GUI of RViz or *mir_teleop*).
      2. Once your robot is at the desired position, you can enter a name and press enter.

        .. note::

          Note : The name of the location should be ALL CAPS.
          For example, CORNER_1, MAIN_DOOR, etc. If the name contains any lower case character,
          the server will not work) You will see pose of the robot inside square brackets in
          the next line and prompted for another name.

      3. Repeat step 1 and 2 to add multiple names to different locations inside the map.
      4. To close this interactive program, press *Ctrl + z*. (Note : *Ctrl + c* won't work.) Then kill this process.

        .. code-block:: bash

          ps

        take a note of the `pid` of python process.

        .. code-block:: bash

          kill -9 <pid_number>

        Now stop *mir_2dnav* and start it again.

        Launch move base launch file (In another terminal)

        .. code-block:: bash

          roslaunch mir_move_base_safe move_base.launch

        Run the server file. (In another terminal)

        .. code-block:: bash

          rosrun mir_move_base_safe move_base_safe_server.py

        You can test everything by running client test file. (In another terminal)

        .. code-block:: bash

          rosrun mir_move_base_safe move_base_safe_client_test.py SOURCE_NAME DESTINATION_NAME

        For example, if you want to move the robot from MAIN_DOOR to CORNER_1, then

        .. code-block:: bash

          rosrun mir_move_base_safe move_base_safe_client_test.py MAIN_DOOR CORNER_1

        .. note::

          The source location is irrelevant for the client test file.
          Your robot can be anywhere and the program will still work correctly.
          Just give some valid location name as a place holder.)
          The client will print **success : True**, if it was able to successfully navigate to the destination position.

RViz setup
-----------


* Add *Map, RobotModel, LaserScan* using the "Add" button in bottom left corner of "Display" section of RViz.

* In *Map*, change the topic to */map*

* In *LaserScan*, change the topic to */scan_front*. Add another *LaserScan* and change its topic to */scan_rear*.

* In global option, change the *Fixed Frame* to *map*.

.. note::

  You can also add another *PoseArray* and change its topic to */move_base/GlobalPlannerWithOrientation*
  to visualise the plan created by the *mir_2Dnav* node.