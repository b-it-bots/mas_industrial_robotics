ARG ROS_DISTRO

FROM osrf/ros:$ROS_DISTRO-desktop-full

ENV IS_CI=1
    
USER root

RUN apt-get update -y -qq \
    && apt upgrade -y -qq \
    && apt-get install -y -qq \
      apt-utils \
      cmake \
      gdb \
      less \
      checkinstall \
      software-properties-common \
      curl \
      figlet \
      wget \
      nmap \
      unzip \
      gcc-multilib \
      g++-multilib \ 
      python-pip 

#update ROS GPG key: https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN pip install --upgrade pip==20.3

RUN apt-get update --fix-missing

# Some QT-Apps/Gazebo don't show controls without this
ENV QT_X11_NO_MITSHM 1

# create robocup user
ARG UNAME=robocup
ARG WS_DIR=/home/$UNAME/$ROS_DISTRO/catkin_ws
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -s /bin/bash -N -u $UID $UNAME && \
    adduser $UNAME sudo && \
    echo "$UNAME ALL=(ALL:ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/$UNAME && \
    mkdir -p $WS_DIR &&\
    chown $UNAME:$GID -R $WS_DIR

WORKDIR $WS_DIR
USER $UNAME
COPY images/mas_industrial_robotics/bashrc /home/$UNAME/.bashrc
COPY images/mas_industrial_robotics/bash_profile /home/$UNAME/.bash_profile

WORKDIR $WS_DIR/src/mas_industrial_robotics
COPY . $WS_DIR/src/mas_industrial_robotics

USER root
RUN chown $UNAME:$GID -R /home/$UNAME

USER $UNAME
# Set Environment variables
ENV ROBOT=youbot-brsu-4
ENV ROBOT_ENV=brsu-c025

RUN ./setup.sh --ros_install base --ros_distro $ROS_DISTRO --ws_dir $WS_DIR --docker 1

#remove cache
USER root
RUN rm -rf /var/lib/apt/lists/*

USER $UNAME
COPY images/mas_industrial_robotics/entrypoint.sh /usr/sbin/entrypoint.sh
RUN sudo chmod +x /usr/sbin/entrypoint.sh
ENTRYPOINT /usr/sbin/entrypoint.sh
