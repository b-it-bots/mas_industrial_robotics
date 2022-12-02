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

    # D435 camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mir_bringup'), 'components'),
            '/realsense_camera.launch.py']),
        launch_arguments={
            'camera_name': 'tower_cam3d',
            'camera_serial': "'827312073531'",
        }.items()
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
        lasers_launch,
        camera_launch,
        gripper_launch,
    ])
