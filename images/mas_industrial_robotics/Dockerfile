ARG ROS_DISTRO

FROM bitbots/bitbots-base:$ROS_DISTRO

ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}

ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV IS_CI=1
    

user root
RUN apt-get update -qq \
    && sudo apt-get install -y -qq \
      apt-utils \
      cmake \
      checkinstall \
      software-properties-common \
      curl \
      figlet \
      wget \
      nmap \
      unzip \
      gcc-multilib \
      g++-multilib \
      python3-pip

RUN apt-get install -y libopencv-dev ros-$ROS_DISTRO-vision-opencv \
        yasm libjpeg-dev libavcodec-dev libavformat-dev libswscale-dev \
        libdc1394-22-dev libv4l-dev python-dev python-numpy libtbb-dev \
        qt5-default libgtk2.0-dev libmp3lame-dev libopencore-amrnb-dev \
        libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev \
        x264 v4l-utils pkg-config

RUN apt-get update --fix-missing

# Install mesa libary for HW acceleration 
# Intel graphic cards
RUN apt-get update &&\
    apt-get install -y \
        libgl1-mesa-glx \
        libgl1-mesa-dri 
  
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

RUN ./setup.sh -ros_install base -ws_dir $WS_DIR -docker 1

COPY images/mas_industrial_robotics/entrypoint.sh /usr/sbin/entrypoint.sh
RUN sudo chmod +x /usr/sbin/entrypoint.sh
ENTRYPOINT /usr/sbin/entrypoint.sh
