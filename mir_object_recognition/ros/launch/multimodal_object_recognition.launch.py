from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  ld = LaunchDescription()

  scene_seg_constraints = os.path.join(
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
    executable="component_container_mt",
    composable_node_descriptions=[
        ComposableNode(
            package="mir_object_recognition",
            plugin="perception_namespace::MultiModalObjectRecognitionROS",
            name="mmor",
            remappings=[
                ("input_image_topic", "/camera/color/image_raw"),
                ("input_cloud_topic", "/camera/depth/color/points"),
            ],
            parameters=[{'objects_info': objects_info}]
        )
    ],
    parameters=[scene_seg_constraints],
    output="screen",
    # prefix=['xterm -e gdb -ex run --args'],
  )
  ld.add_action(container)
  return ld
