FROM ros:kinetic

RUN apt update -qq && \
    apt install -y -qq sudo software-properties-common && \
    apt install -y -qq gcc-multilib g++-multilib

WORKDIR /mas_industrial_robotics
COPY . /mas_industrial_robotics

# Set Environment variables
ENV ROBOT=youbot-brsu-4
ENV ROBOT_ENV=brsu-c025

RUN ./setup.sh

WORKDIR /
