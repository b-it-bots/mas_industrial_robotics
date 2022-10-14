#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():

    robot_name = os.environ['ROBOT']

    joypad_config_path = os.path.join(
        get_package_share_directory('mir_teleop'),
        'ros', 'config',
        'teleop.yaml')
    joy_config_path = os.path.join(
        get_package_share_directory('mir_teleop'),
        'ros', 'config',
        'joy.yaml')

    arm_config_path = os.path.join(
        get_package_share_directory('mir_hardware_config'),
        robot_name,
        'config',
        'arm.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config_path],
            arguments=[('__log_level:=debug')]
    )

    mir_teleop_node = Node(
            package='mir_teleop',
            executable='teleop_joypad_node',
            name='mir_teleop_joypad',
            output='screen',
            parameters=[joypad_config_path, arm_config_path],
            # prefix=['xterm -e gdb -ex run --args'],
#            remappings=[('cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        joy_node,
        mir_teleop_node
    ])
