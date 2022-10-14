#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os
import xacro


def generate_launch_description():

    robot_name = os.environ['ROBOT']

    # planning_context
    youbot_xacro_file = os.path.join(get_package_share_directory('mir_hardware_config'), robot_name, 'urdf',
                                     'robot.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', youbot_xacro_file])

    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     arguments=[youbot_xacro_file],
    # )

    robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('mir_bringup'), 'robots',),
                f'/{robot_name}.launch.py'])
    )

    return LaunchDescription([
        robot_state_publisher,
        robot_launch,
        # joint_state_publisher,
    ])
