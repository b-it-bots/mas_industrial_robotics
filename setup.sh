#!/bin/bash

# Exit immediately if any command exits with a nonzero exit value
set -e

function install_basic_packages {
    sudo apt-get update -qq
    sudo apt-get install -y -qq curl figlet
}

function fancy_print {
    echo "################################################################################"
    figlet -t $1
    echo "################################################################################"
}

# Install ROS Melodic
function update_keys {
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
}

function install_ros_melodic_base {
    sudo apt update -qq
    sudo apt install -y -qq ros-melodic-ros-base
}

function install_ros_dependencies {
    sudo apt-get install python-rosdep
    # sudo rm -rf /etc/ros/rosdep/sources.list.d/*
    sudo rosdep init -q
    rosdep update -q
    sudo apt install -y -qq python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools python-pip
    sudo pip install catkin_pkg empy
    sudo rm -rf /var/lib/apt/lists/*
}

function install_ros {
    update_keys
    if $1
        then
            fancy_print "Installing ROS"
            install_ros_melodic_base
    fi
    fancy_print "Installing ROS Dependencies"
    install_ros_dependencies
    source /opt/ros/melodic/setup.bash
}

# Setup catkin workspace in the home directory
function setup_catkin_ws {
    fancy_print "Setting up a catkin workspace"
    mkdir -p $INSTALL_DIR/catkin_ws/src
    cd $INSTALL_DIR/catkin_ws
    catkin init
    catkin config --extend /opt/ros/melodic/
    catkin build
    source $INSTALL_DIR/catkin_ws/devel/setup.bash
}

# Clone and install mas_industrial_robotics and all related dependencies
function get_mas_industrial_robotics {
    fancy_print "Getting source code"
    cd $INSTALL_DIR/catkin_ws/src
    cp -r $ROOT_DIR ~/catkin_ws/src
    cd $INSTALL_DIR/catkin_ws/src/mas_industrial_robotics
    ./repository.debs
    sudo rm -rf /var/lib/apt/lists/*
}

function build_mas_industrial_robotics {
    fancy_print "Building ROS packages"
    source $INSTALL_DIR/catkin_ws/devel/setup.bash
    # Disable building the youbot_driver_ros_interface in travis CI as it expects a user input during build
    touch $INSTALL_DIR/catkin_ws/src/youbot_driver_ros_interface/CATKIN_IGNORE
    catkin build
}


# Begin shell command executions from here
# Store the root directory path in a variable
ROOT_DIR=$(pwd)

FULL_INSTALL=false
if [ $# -eq 1 ] && [ $1 == "full" ]
    then
        FULL_INSTALL=true
fi

# If no arg then catkin_ws is created in home
# folder, otherwise in specified folder
if [ -z "$2" ]
    then
        INSTALL_DIR=~
    else
        INSTALL_DIR=$2
fi

install_basic_packages
install_ros $FULL_INSTALL
setup_catkin_ws
get_mas_industrial_robotics
build_mas_industrial_robotics
fancy_print "Build Complete"
