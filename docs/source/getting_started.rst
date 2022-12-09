.. _getting_started:

Getting started
###############

.. _install_ubuntu:

Install Ubuntu
==============

The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Rolling: Ubuntu 22.04

If you do not have a Ubuntu distribution on your computer you can download it `here <http://www.ubuntu.com/download>`_.

.. _git_version_control:

Git - Version Control
=====================

* Install Git Software

  Install the Git core components and some additional GUI's for the version control:

  .. code-block:: bash

    sudo apt-get install git-core gitg gitk

* Set Up Git

  Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

  .. code-block:: bash

     git config --global user.name "Your Name Here"

  Git also saves your email address into the commits you make.

  .. code-block:: bash

     git config --global user.email "your-email@youremail.com"


* GIT Tutorial
  If you have never worked with git before, we recommend to go through the
  `basic git tutorial <http://excess.org/article/2008/07/ogre-git-tutorial/>`_.

.. _getting_started_docker:

.. _robot_operating_system:

ROS - Robot Operating System
============================

* Install ROS2

  The repository has been tested successfully with ROS2 Rolling distribution.
  Use the link behind a ROS2 distribution to get to the particular
  `ROS Rolling installation instructions <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html>`_.

  .. note::
    Do not forget to update your .bashrc!

* ROS2 Tutorials

  If you have never worked with ROS before, we recommend to go through
  `the beginner tutorials provided by ROS <https://docs.ros.org/en/rolling/Tutorials.html>`_.

  In order to understand at least the different core components of ROS2, you have to complete Beginner tutorials.

.. _setup_catkin_workspace:

Setup catkin workspace
=========================

* Create a catkin workspace

  .. code-block:: bash

    source /opt/ros/rolling/setup.bash
    mkdir -p ~/rolling/src
    cd ~/rolling/src

* Clone and compile the MAS industrial robotics software

  First of all you have to clone the repository.

  .. code-block:: bash

    cd ~/noetic/src;
    git clone -b humble https://github.com/mas_industrial_robotics.git

  Then go on with installing further external dependencies:

  .. code-block:: bash

    cd ~/rolling/src/
    vcs import < mas_industrial_robots/mir.repos

  The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.

  And finally compile the repository:

  .. code-block:: bash

    cd ~/rolling
    colcon build
    source ~/rolling/install/setup.bash

  If no errors appear everything is ready to use. Great job!

* Setting the Environment Variables

  * ROBOT variable

    With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line will add the variable to your .bashrc:

    .. code-block:: bash

      echo "export ROBOT=youbot-brsu-2" >> ~/.bashrc
      source ~/.bashrc

.. _bringup_robot:

Bring up the robot and its basic components
===========================================

* At the Real Robot

  .. code-block:: bash

     ros2 launch mir_bringup robot.launch

* Visualize the robot state and sensor data

  .. code-block:: bash

     rviz2