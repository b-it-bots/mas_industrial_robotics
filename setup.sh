#!/bin/bash

function install_basic_packages {
    sudo apt-get update -qq
    sudo apt-get install -y -qq curl figlet
}

function fancy_print {
    echo "################################################################################"
    figlet -t $1
    echo "################################################################################"
}

# Install ROS Kinetic
function update_keys {
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu xenial main\" > /etc/apt/sources.list.d/ros-latest.list"
    sudo curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add - 
}

function install_ros_kinetic_base {
    sudo apt-get update -qq
    sudo apt-get install -y -qq ros-kinetic-ros-base
}

function install_ros_dependencies {
    sudo rosdep init -q
    sudo rosdep update -q
    sudo apt install -y -qq python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools python-pip
    sudo pip install catkin_pkg empy
}

function install_ros {
    fancy_print "Installing ROS"
    update_keys
    install_ros_kinetic_base
    install_ros_dependencies
    source /opt/ros/kinetic/setup.bash
}

# Setup catkin workspace in the home directory
function setup_catkin_ws {
    fancy_print "Setting up a catkin workspace"
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin config --extend /opt/ros/kinetic/
    catkin build
    source ~/catkin_ws/devel/setup.bash
}

# Clone and install mas_industrial_robotics and all related dependencies
function get_mas_industrial_robotics {
    fancy_print "Getting source code"
    cd ~/catkin_ws/src
    cp -r $ROOT_DIR ~/catkin_ws/src
    cd mas_industrial_robotics
    ./repository.debs
}

function build_mas_industrial_robotics {
    fancy_print "Building ROS packages"
    source ~/catkin_ws/devel/setup.bash
    # Disable building the youbot_driver_ros_interface and mcr_arm_cartesian_control in travis CI
    touch ~/catkin_ws/src/youbot_driver_ros_interface/CATKIN_IGNORE
    touch ~/catkin_ws/src/mas_common_robotics/mcr_manipulation/mcr_arm_cartesian_control/CATKIN_IGNORE
    catkin build 
}


# Begin shell command executions from here
# Store the root directory path in a variable
ROOT_DIR=$(pwd)

install_basic_packages
install_ros
setup_catkin_ws
get_mas_industrial_robotics
build_mas_industrial_robotics
fancy_print "Build Complete"
