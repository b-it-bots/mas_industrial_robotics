.. _mir_navigation:

Navigation
#############

On workstation or your PC
==========================

* To shh the youbot (in all terminals):

  .. code-block:: bash

      yb4

  .. note::

      alias yb4=ssh -X robocup@youbot-brsu-4-pc2

* Export the youbot ssh alias

  .. code-block:: bash

      export_yb4

  .. note::

      alias export_yb4=export ROS_MASTER_URI=http://youbot-brsu-4-pc2:11311

* Run rviz

  .. code-block:: bash

    rosrun rviz rviz

  Set the global frame to `base_link`

.. _2d_slam:

2D SLAM
========

* Run roscore

  .. code-block:: bash

      roscore

* Launch the robot

  .. code-block:: bash

      roslaunch mir_bringup robot.launch

* Run 2D SLAM

  .. code-block:: bash

      roslaunch mir_2dslam 2dslam.launch

  .. note::

      The map is built using the front laser's only

* Run the map saver

  Go to the map configuration directory

  .. code-block:: bash

      roscd mcr_default_env_config

  By using `ls` you can see several folders corresponding to existing environments.
  You can either use an existing map or create a new one:

  .. code-block:: bash

      mkdir [map_name]
      cd [map_name]

  And then run:

  .. code-block:: bash

      rosrun map_server map_saver

  This will create two files: a `map.pgm` and `map.yml`.

  Finally, to use the map that you just created you need to check which map will be loaded by the navigation stack:

  .. code-block:: bash

      echo $ROBOT_ENV

  If you need to change it:

  .. code-block:: bash

      export ROBOT_ENV=[map_name]

  .. note::

      Usually the `.rosc` script is used to set the environment, among other variables.

.. _2d_navigation:

2D Navigation
================

* Bringup the robot

  First export the environment to be used:

  .. code-block:: bash

      export ROBOT_ENV=brsu-C025

  Launch the robot:

  .. code-block:: bash

      roslaunch mir_bringup robot.launch

* Launch the navigation node

  .. code-block:: bash

      roslaunch mir_2dnav 2dnav.launch

* Launch the planning bringup

  .. code-block:: bash

      roslaunch mir_planning_bringup robot.launch

* Create navigation goals and orientations

  First you need to create the files where goals will be saved:

  .. code-block:: bash

    touch navigation_goals.yaml
    touch orientation_goals.yaml

* Localize the robot

  In rviz:

  1. Select the 2D pose estimate
  2. Click the position near the robot
  3. Move with joystick
  4. Launch navigation tools in yb2

* Save the navigation and orientation goals

  .. code-block:: bash

      roscd mcr_default_env_config
      cd brsu-C025
      rosrun mcr_navigation_tools save_base_map_poses_to_file

* Test navigation goal using move_base

.. code-block:: bash

    rosrun mir_move_base_safe move_base_safe_server.py
    rosrun mir_move_base_safe move_base_safe_client_test.py [dest]

* Navigation test using refbox

.. code-block:: bash

    roslaunch mir_basic_navigation_test refbox_parser.py

.. _Direct_robot_control:

Direct base controller
======================

The direct base controller is used to perform relative motions of the robot with respect to a reference frame. It has an optional feature to stop the motions when obstacles are detected with the laser scanners.

* Bringup the robot

.. code-block:: bash

      roslaunch mir_bringup robot.launch

* Launch the direct base controller

.. code-block:: bash

    roslaunch mir_direct_base_controller direct_base_controller.launch


* Launch the pose mockup GUI on your PC:
.. code-block:: bash

    export ROS_MASTER_URI=http://<robot_ip_address>:11311
    roslaunch mir_direct_base_controller pose_mock_up_gui.launch

A GUI pop up window will appear.
Set the relative pose accordingly in the window (e.g. 0.1 in X to move forward 10 cm).

* Run rviz on your PC:

  .. code-block:: bash

    export ROS_MASTER_URI=http://<robot_ip_address>:11311
    rviz

  Set the 'Fixed Frame' to `odom`
  Add a 'Pose' display and set the topic to '/mcr_navigation/direct_base_controller/input_pose'


* Publish an `e_start` event to the direct base controller node
.. code-block:: bash

    rostopic pub /mcr_navigation/direct_base_controller/coordinator/event_in std_msgs/String "data: 'e_start'"
