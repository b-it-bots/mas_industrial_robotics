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

## Errors/problems encountered during build and their possible solutions
### Unable to access 'unmerged_packages_for_testing' repo
This is a private repository. If you do not have access to this repo, running ''./repository.debs'' will return an error.

_Solution_: Request access from Deebul Nair or Santosh Thoduka.

### Package 'costmap_2d' takes too long to build
This is experienced on VM setups.

_Solution_: Try building only this package. Generally takes around 16-20 minutes depending on PC configuration.

     catkin build costmap_2d

### Error related to 'KDL'
When building package ''mcr_arm_cartesian_control'', following error could be encountered:

     int error_sigma = ((KDL::ChainIkSolverVel_wdls*) ik_solver)->getSigma(sigma);

_Solution_: As suggested in this [thread], comment the line in the source file located in folder ''mcr_manipulation/mcr_arm_cartesian_control/common/src/arm_cartesian_control.cpp''.

[thread]: https://github.com/b-it-bots/mas_common_robotics/commit/00ce574505b9571fd0299e0684790341c1b8fdf4

### Errors related to uninstalled packages
There are certain packages needed as prerequisite. 

_Solution_: In case you do not have those, please google the relevant package, clone the git repo to your local machine, then build it. Following packages are generally found to be missing.

#### [brics_actuator]
[brics_actuator]: http://wiki.ros.org/brics_actuator
Clone the git repo to your machine:

     git clone git@github.com:wnowak/brics_actuator.git

Build the package:

     roscd && cd ../
     catkin build brics_actuator

Copy the header files to folder ''/opt/ros/kinetic/include/''. This is required to be able to reference the header files from other source files, else ''...file not found'' error is encountered.

     roscd && cd include/
     sudo cp -R brics_actuator/ /opt/ros/kinetic/include/

Then continue building remaining packages

     roscd && cd ../
     catkin build

#### [bfl]
[bfl]: http://wiki.ros.org/bfl
Clone the git repo to your machine:

     git clone git@github.com:ros-gbp/bfl-release.git

Build the package:

     roscd && cd ../
     catkin build bfl

Then continue building remaining packages

     roscd && cd ../
     catkin build

#### [geometry2]
[geometry2]: http://wiki.ros.org/geometry2
Clone the git repo to your machine:

     git clone git@github.com:ros/geometry2.git

Build the package:

     roscd && cd ../
     catkin build geometry2

Then continue building remaining packages

     roscd && cd ../
     catkin build

#### [laser_filters]
[laser_filters]: http://wiki.ros.org/laser_filters
Clone the git repo to your machine:

     git clone git@github.com:ros-perception/laser_filters.git

Build the package:

     roscd && cd ../
     catkin build laser_filters

Then continue building remaining packages

     roscd && cd ../
     catkin build

### Error related to 'whole_body_motion_control' package
While building this package, an error is encountered in the CMake file, line 66.

     src/whole_body_motion_control/whole_body_motion_control_utils.cpp

This file is not available in the folder ''src/''.

_Solution_: Open this CMake file and comment out the above line.

