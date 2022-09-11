"""
Copyright 2022 Bonn-Rhein-Sieg University

Author: Vivek Mannava, Vamsi Kalagaturu

"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  ld = LaunchDescription()
  # declare the launch arguments
  log_directory = LaunchConfiguration('log_directory')
  log_directory_arg = DeclareLaunchArgument('log_directory',
                                            default_value='/tmp/', 
                                            description='Log directory to store the logs, debug images and point clouds')
  
  config = os.path.join(
    get_package_share_directory('mir_object_recognition'),
    'ros',
    'config',
    'scene_segmentation_constraints.yaml'
  )

  objects_info = os.path.join(
    get_package_share_directory('mir_object_recognition'),
    'ros',
    'config',
    'objects.yaml'
  )

  container = ComposableNodeContainer(
    name="MMOR_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[

        ComposableNode(
            package="mir_object_recognition",
            plugin="perception_namespace::DataCollector",
            name="data_collector",
            remappings=[
                    ("input_image_topic", "/camera/color/image_raw"),
                    ("input_cloud_topic", "/camera/depth/color/points"),
            ],
            parameters=[{'objects_info': objects_info}, {'logdir': log_directory}],
        )
    ],
    parameters=[config],
    output="screen",
    # prefix=['xterm -e gdb -ex run --args'],
  )
  ld.add_action(log_directory_arg)
  ld.add_action(container)
  return ld
