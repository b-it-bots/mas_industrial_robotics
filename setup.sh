#!/bin/bash

# Exit immediately if any command exits with a nonzero exit value
set -e

function install_basic_packages {
    sudo apt-get update -qq
    sudo apt-get install -y -qq curl figlet wget
}

function fancy_print {
    echo "################################################################################"
    figlet -t $1
    echo "################################################################################"
}

# Install ROS Melodic
function update_keys {
    if [ $DOCKER_INSTALL = 0 ];
      then
        echo "Setting up keys"
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
        sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
        #sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -sc) main" -u
    fi
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
}

# not required in docker
function install_ros_base {
    sudo apt update -qq
    if [[ $ROS_INSTALL = "full" ]]
        then
            echo "Installing ROS desktop full"
            sudo apt install -y -qq ros-$ROS_DISTRO-desktop-full
    elif [[ $ROS_INSTALL = "base" ]]
        then
            echo "Installing ROS base"
            sudo apt install -y -qq ros-$ROS_DISTRO-ros-base
    fi
}

function install_ros_dependencies {
    sudo rm -rf /etc/ros/rosdep/sources.list.d/*
    sudo rosdep init -q
    rosdep update -q
    sudo apt install -y -qq python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools python-pip
    sudo pip install catkin_pkg empy
    #sudo rm -rf /var/lib/apt/lists/*
}

function install_perception_dependencies {
    fancy_print "Installing perception dependencies"
    sudo pip install --no-cache-dir --ignore-installed enum34
    sudo pip install --no-cache-dir -U tensorflow==1.14.0
    sudo pip install --no-cache-dir -U scikit-learn easydict joblib
}

function install_ros {
    update_keys
    fancy_print "Installing ROS"
    install_ros_base
    fancy_print "Installing ROS Dependencies"
    install_ros_dependencies
    source /opt/ros/$ROS_DISTRO/setup.bash
}

# Setup catkin workspace in the home directory
function setup_catkin_ws {
    fancy_print "Setting up a catkin workspace"
    
    if [ $DOCKER_INSTALL = 0 ];
      then
        echo "Create src directory in $WS_DIR"
        mkdir -p $WS_DIR/src
    fi

    cd $WS_DIR
    catkin init
    catkin config --extend /opt/ros/$ROS_DISTRO/
    #catkin build
    #source $WS_DIR/devel/setup.bash
}

# Clone mas_industrial_robotics and other repos
function get_mas_industrial_robotics {
    fancy_print "Getting source code"
    wstool init --shallow $WS_DIR/src
    wstool merge -t $WS_DIR/src $ROOT_DIR/repository.rosinstall
    wstool update -t $WS_DIR/src
    if [ $DOCKER_INSTALL = 0 ];
      then
        echo "Copy mas industrial robotics to src"
        cp -r $ROOT_DIR $WS_DIR/src
    fi
}

# Install ros and other dependencies
function install_mas_dependencies {
    fancy_print "Installing MAS Dependencies"
    rosdep update -q
    sudo apt-get update
    rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO --skip-keys rosplan_demos -y
}

function build_mas_industrial_robotics {
    fancy_print "Building ROS packages"
    #source $WS_DIR/devel/setup.bash
    # Disable building the youbot_driver_ros_interface in travis CI as it expects a user input during build
    touch $WS_DIR/src/youbot_driver_ros_interface/CATKIN_IGNORE
    # Build mercury planner first for CI to avoid crashing due to socket conn error
    catkin build mercury_planner
    catkin build
}

ROS_INSTALL=base
ROS_DISTRO=melodic
WS_DIR=""
DOCKER_INSTALL=0

while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "Usage: bash setup.sh"
      echo " "
      echo "options:"
      echo "-h, --help                            show brief help"
      echo "-ri, --ros_install (str)              whether to install desktop-full or base (default: base)"
      echo "-rd, --ros_distro (str)               whether to install desktop-full or base (default: melodic)"
      echo "-ws, --ws_dir  (str)                  workspace dir"
      echo "-d, --docker (0 or 1)                 whether to install in docker mode or system wide (default: 0 / system wide)"
      exit 0
      ;;
    -ri|--ros_install)
      ROS_INSTALL="$2"
      shift
      shift
      ;;
    -rd|--ros_distro)
      ROS_DISTRO="$2"
      shift
      shift
      ;;
    -ws|--ws_dir)
      WS_DIR="$2"
      shift
      shift
      ;;
    -d|--docker)
      DOCKER_INSTALL=$2
      shift
      shift
      ;;
    *)
      break
      ;;
  esac
done

# Begin shell command executions from here
# Store the root directory path in a variable
ROOT_DIR=$(pwd)

install_basic_packages
install_ros 
setup_catkin_ws
get_mas_industrial_robotics
install_mas_dependencies
install_perception_dependencies
build_mas_industrial_robotics
fancy_print "Build Complete"
