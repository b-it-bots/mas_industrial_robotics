#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    youbot_oodl_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_bringup'), 'components'),
            '/youbot_oodl_driver.launch.py']),
        launch_arguments={
            'youBotHasBase': 'True',
            'youBotHasArms': 'True',
        }.items()
    )

    lasers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_bringup'), 'components'),
            '/urg_node.launch.py'])
    )

    teleop_joypad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_teleop'), 'ros', 'launch'),
            '/teleop_joypad.launch.py'])
    )

    return LaunchDescription([
        youbot_oodl_driver_launch,
        teleop_joypad_launch,
        lasers_launch,
    ])
