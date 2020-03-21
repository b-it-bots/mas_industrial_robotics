.. _getting_started:

Getting started
###############

.. _install_ubuntu:

Install Ubuntu
==============

The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Kinetic: Ubuntu 16.04

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

.. _robot_operating_system:

ROS - Robot Operating System
============================

* Install ROS

  The repository has been tested successfully with the following ROS distributions. 
  Use the link behind a ROS distribution to get to the particular 
  `ROS Kinetic installation instructions <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.

  .. note::
    Do not forget to update your .bashrc!

* ROS Tutorials
  
  If you have never worked with ROS before, we recommend to go through 
  `the beginner tutorials provided by ROS <http://wiki.ros.org/ROS/Tutorials>`_.

  In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 

.. _setup_catkin_workspace:

Setup catkin workspace
=========================

* Create a catkin workspace

  .. code-block:: bash

    source /opt/ros/kinetic/setup.bash
    mkdir -p ~/kinetic/src; cd ~/kinetic/src
    catkin_init_workspace
    catkin build

* Clone and compile the MAS industrial robotics software

  First of all you have to clone the repository.

  .. code-block:: bash

    cd ~/kinetic/src;
    git clone git@github.com:b-it-bots/mas_industrial_robotics.git

  Then go on with installing further external dependencies:

  .. code-block:: bash

    cd ~/kinetic/src/mas_industrial_robotics
    ./repository.debs
    source ~/kinetic/devel/setup.bash

  The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.

  And finally compile the repository:

  .. code-block:: bash

    cd ~/kinetic
    catkin build

  If no errors appear everything is ready to use. Great job!

* Setting the Environment Variables
  
  * ROBOT variable
    
    With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line will add the variable to your .bashrc:

    .. code-block:: bash

      echo "export ROBOT=youbot-brsu-1" >> ~/.bashrc
      source ~/.bashrc

  * ROBOT_ENV Variable

    The ROBOT_ENV variable can be used to switch between different environments. The following line will add the variable to your .bashrc:
    
    * Real robot
    
    .. code-block:: bash

      echo "export ROBOT_ENV=brsu-c025" >> ~/.bashrc
      source ~/.bashrc
  
    * Simulation
    
    .. code-block:: bash

      echo "export ROBOT_ENV=brsu-c025-sim" >> ~/.bashrc
      source ~/.bashrc

.. _bringup_robot:

Bring up the robot and its basic components
===========================================

* In Simulation

  .. code-block:: bash

     roslaunch mir_bringup_sim robot.launch
        
  In a new terminal you can open the Gazebo GUI to see the environment and the robot

  .. code-block:: bash
	    
     rosrun gazebo_ros gzclient

* At the Real Robot

  .. code-block:: bash

     roslaunch mir_bringup robot.launch
     

* Test the base

  .. code-block:: bash

     roslaunch mir_teleop teleop_keyboard.launch
     

* Visualize the robot state and sensor data

  .. code-block:: bash

     rosrun rviz rviz


* Build a map for base navigation

  .. code-block:: bash

     roslaunch mir_2dslam 2dslam.launch
     

* Use autonomous navigation
  
  * Omni-directional navigation

    .. code-block:: bash

        roslaunch mir_2dnav 2dnav.launch nav_mode:=dwa

Click on the menu bar "File -> Open Config", navigate to "~/indigo/src/mas_industrial_robotics" and select the "youbot.rviz" file.