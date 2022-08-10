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

      roslaunch mir_2dnav 2Dnav.launch

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
      rosrun mcr_navigation_tools save_map_poses_to_file

* Test navigation goal using move_base

.. code-block:: bash

    rosrun mir_move_base_safe move_base_safe_server.py
    rosrun mir_move_base_safe move_base_safe_client_test.py [source] [dest]

* Navigation test using refbox

.. code-block:: bash

    roslaunch mir_basic_navigation_test refbox_parser.py