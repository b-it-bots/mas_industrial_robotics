#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():

    front_laser_config_path = os.path.join(
        get_package_share_directory('mir_teleop'),
        'ros', 'config',
        'front_laser_config.yaml')

    rear_laser_config_path = os.path.join(
        get_package_share_directory('mir_teleop'),
        'ros', 'config',
        'rear_laser_config.yaml')

    front_urg_node = Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node_driver_front',
            output='screen',
            parameters=[front_laser_config_path],
            remappings=[('scan', '/scan_front')]
    )

    rear_urg_node = Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node_driver_rear',
            output='screen',
            parameters=[rear_laser_config_path],
            remappings=[('scan', '/scan_rear')]
    )

    return LaunchDescription([
        front_urg_node,
        rear_urg_node
    ])
