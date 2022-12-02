#!/usr/bin/env python3

# launch file for the teensy dynamixel gripper

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():

    # add the gripper controller node
    gripper_controller = Node(
        package='mir_gripper_controller',
        executable='gripper_controller',
        name='gripper_controller',
        output='both',
        remappings=[('joint_state', '/joint_states')]
    )

    return LaunchDescription([
        gripper_controller,
    ])