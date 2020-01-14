[![Build Status](https://travis-ci.com/b-it-bots/mas_industrial_robotics.svg?branch=kinetic)](https://travis-ci.com/b-it-bots/mas_industrial_robotics)

## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Kinetic: Ubuntu 16.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make.

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.


- ROS Kinetic - http://wiki.ros.org/kinetic/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc!
  

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://wiki.ros.org/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 


## Set up a catkin workspace

    source /opt/ros/kinetic/setup.bash
    mkdir -p ~/kinetic/src; cd ~/kinetic/src
    catkin_init_workspace
    catkin build
    
## Clone and compile the MAS industrial robotics software
First of all you have to clone the repository.

    cd ~/kinetic/src;
    git clone git@github.com:b-it-bots/mas_industrial_robotics.git


Install documentation generation software
    cd mas_industrial_robotics
    git submodule init
    git submodule update
    pip install --user sphinx-rtd-theme

Then go on with installing further external dependencies:

    cd ~/kinetic/src/mas_industrial_robotics
    ./repository.debs

    source ~/kinetic/devel/setup.bash

The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.


And finally compile the repository:

    cd ~/kinetic
    catkin build


If no errors appear everything is ready to use. Great job!


### Setting the Environment Variables
#### ROBOT variable
With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line will add the variable to your .bashrc:

     echo "export ROBOT=youbot-brsu-1" >> ~/.bashrc
     source ~/.bashrc



#### ROBOT_ENV Variable
The ROBOT_ENV variable can be used to switch between different environments. The following line will add the variable to your .bashrc:
#### Real robot
     echo "export ROBOT_ENV=brsu-c025" >> ~/.bashrc
     source ~/.bashrc
#### Simulation
     echo "export ROBOT_ENV=brsu-c025-sim" >> ~/.bashrc
     source ~/.bashrc



## Bring up the robot and it's basic components
### In Simulation

     roslaunch mir_bringup_sim robot.launch
     
     
In a new terminal you can open the Gazebo GUI to see the environment and the robot

	     rosrun gazebo_ros gzclient

### At the Real Robot

     roslaunch mir_bringup robot.launch
     

## Test the base

     roslaunch mir_teleop teleop_keyboard.launch
     

## Visualize the robot state and sensor data

     rosrun rviz rviz


## Build a map for base navigation

     roslaunch mir_2dslam 2dslam.launch
     

## Use autonomous navigation
### Omni-directional navigation

     roslaunch mir_2dnav 2dnav.launch nav_mode:=dwa

     


Click on the menu bar "File -> Open Config", navigate to "~/indigo/src/mas_industrial_robotics" and select the "youbot.rviz" file.
