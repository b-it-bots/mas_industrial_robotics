"""
Copyright 2022 Bonn-Rhein-Sieg University

Author: Vamsi Kalagaturu, Vivek Mannava

"""
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os

def generate_launch_description():
  ld = LaunchDescription()
  node_name = "mmor"

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
            name=node_name,
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

  # transition mmor node to shutdown before SIGINT
  shutdown_event = RegisterEventHandler(
    OnShutdown(
      on_shutdown=[
        EmitEvent(event=ChangeState(
          lifecycle_node_matcher=matches_node_name(node_name=node_name),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
        )),
        LogInfo(
            msg="[mmor_launch] mmor node is exiting."),
      ],
    )
  )

  ld.add_action(shutdown_event)
  ld.add_action(container)
  return ld
