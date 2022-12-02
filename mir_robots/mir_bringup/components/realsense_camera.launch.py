#!/usr/bin/env python3

# launch file for realsense camera

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_context import LaunchContext
import os

def generate_launch_description():
    
    # camera name argument
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='tower_cam3d',
        description='Name of the camera')
    
    # camera serial number argument
    declare_camera_serial = DeclareLaunchArgument(
        'camera_serial',
        default_value="'827312073531'",
        description='Serial number of the camera')

    # include realsense launch file with configuration for pointcloud
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_no': LaunchConfiguration('camera_serial'),
            'pointcloud.enable': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'pointcloud.ordered_pc': 'true',
        }.items()
    )

    return LaunchDescription([
        declare_camera_name,
        declare_camera_serial,
        realsense_launch,
    ])