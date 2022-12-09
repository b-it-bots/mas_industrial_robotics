#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('mir_twist_mux'),
                                                            'config',
                                                            'params.yaml')

    declare_cmd_vel_out = DeclareLaunchArgument(
        'cmd_vel_out',
        default_value='/cmd_vel',
        description='Output topic for cmd_vel')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[config_file],
        remappings=[('cmd_vel_prio_high', '/cmd_vel_prio_high'),
                    ('cmd_vel_prio_med', '/cmd_vel_prio_med'),
                    ('cmd_vel_prio_low', '/cmd_vel_prio_low')]
    )

    return LaunchDescription([
        declare_cmd_vel_out,
        twist_mux_node
    ])