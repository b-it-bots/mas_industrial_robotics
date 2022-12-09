#!/usr/bin/env python3

"""

Copyright 2022 Bonn-Rhein-Sieg University

Author: Vamsi Kalagaturu

"""
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
            'youBotHasBase': 'False',
            'youBotHasArms': 'True',
        }.items()
    )

    teleop_joypad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_teleop'), 'ros', 'launch'),
            '/teleop_joypad.launch.py'])
    )

    # gripper launch file
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_gripper_controller'), 'launch'),
            '/teensy_dynamixel_gripper.launch.py'])
    )

    return LaunchDescription([
        youbot_oodl_driver_launch,
        teleop_joypad_launch,
        gripper_launch,
    ])
