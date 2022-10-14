#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():
    config_path = os.path.join(
            get_package_share_directory('youbot_driver'),
            'config')
    youbot_driver_node = Node(
            package='youbot_driver_ros_interface',
            executable='youbot_driver_ros_interface',
            name='youbot_driver_ros_interface',
            output='screen',
            parameters=[
                {"youBotHasBase": True},
                {"youBotHasArms": True},
                {"youBotDriverCycleFrequencyInHz": 50.0},
                {"youBotDriverGripperReadingsCycleFrequencyInHz": 1.0},
                {"youBotConfigurationFilePath": config_path},
                {"trajectoryActionServerEnable": True},
                {"trajectoryPositionGain": 5.0},
                {"trajectoryVelocityGain": 0.0},
                {"youBotBaseName": "youbot-base"},
                {"youBotArmName1": "youbot-manipulator"}
                ],
            remappings=[('/base/joint_states', 'joint_states'),
                        ('arm_1/joint_states', 'joint_states')]
    )
    return LaunchDescription([
        youbot_driver_node
    ])
